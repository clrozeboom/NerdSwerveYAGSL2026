package frc.robot.subsystems.drive.riobridge;

import frc.robot.subsystems.drive.GyroIO;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

/**
 * {@link GyroIO} backed by the RioBridge's navX -- the drop-in replacement for {@code
 * GyroIONone}/{@code GyroIOOnboard} the root README's comments in {@code RobotContainer} were
 * left waiting on. {@code connected} is driven by Attitude-frame staleness at a 100 ms threshold,
 * the same threshold the RioBridge project's own hardware-verification guide uses.
 *
 * <p>This project's {@code GyroIO} (unlike the AdvantageKit {@code spark_swerve} template
 * RioBridge's own {@code core-integration/} was written against) has no per-sample odometry
 * arrays -- {@code Drive} reads one {@code yawPosition} per loop rather than an array of
 * timestamped samples -- so this only ever reports the latest Attitude sample, not the full
 * history. {@link RioBridgeCan} doesn't buffer multiple samples either (see its class javadoc for
 * why), so there's nothing to drain here -- unlike the stream-session version of this class,
 * which had to drain {@code RioBridgeCanDemux}'s pending-samples list every loop just to keep it
 * from growing unbounded.
 *
 * <p><b>Zeroing:</b> the RioBridge protocol is deliberately one-way -- the Core only ever
 * receives frames, it can't send the roboRIO a "zero your yaw" command -- so unlike {@code
 * GyroIOOnboard}, {@link #resetYaw()} can't reset anything on the actual navX. It instead
 * subtracts a Core-side offset before reporting {@code yawPosition}, captured from the last raw
 * reading at the moment {@link #resetYaw()} is called. This matters here specifically because
 * {@code Drive.updateOdometry()} takes a connected gyro's {@code yawPosition} as absolute truth
 * every loop rather than integrating a delta from it -- with no offset applied on this end,
 * {@code Drive.zeroHeading()} would set its pose's rotation to zero for exactly one loop before
 * the next {@code updateInputs} call overwrote it with the RioBridge's unzeroed yaw again.
 *
 * <p><b>Sign convention:</b> the navX reports yaw clockwise-positive and the RioBridge passes
 * {@code AHRS.getYaw()} across the wire untouched, while WPILib is counter-clockwise-positive
 * everywhere. {@link #yawFromNavx} and {@link #yawRateFromNavx} flip it on the way in. Without
 * that, a commanded counter-clockwise spin logs as a negative yaw rate -- measured at -1.404 rad/s
 * against a commanded +1.191 -- and every heading-dependent thing downstream runs backwards:
 * field-oriented drive steers the wrong way as the robot turns, and odometry curves the pose off
 * the opposite side of the field.
 *
 * <p><b>Scale factor:</b> this navX2 reports roughly 10% more rotation than actually happens, and
 * keeps doing so after a recalibration. Turning the robot through one full turn by hand logged
 * 396.31 degrees, and a bench check put it at about 10 degrees extra per 90. {@link #YAW_SCALE}
 * divides that back out. This is compensation for a sensor that is out of spec, not a tune: a
 * navX2 should be good to a fraction of a percent, so treat the number here as a stopgap and the
 * IMU itself as suspect -- worth a firmware check, a look at its mount-orientation configuration,
 * or a replacement.
 *
 * <p>The scale has to be applied to a <i>continuous</i> angle, which is why this class integrates
 * the raw yaw itself rather than scaling what arrives. The navX wraps at +/-180, and scaling a
 * wrapped reading is wrong the moment it wraps: 400 degrees of real rotation arrives as 80, and
 * 80 * 0.908 is 72.6 where the right answer is 40. Accumulating the wrapped deltas first and
 * scaling the running total gets both right.
 *
 * <p>Construct one {@link RioBridgeCan} per robot (it owns the CAN reads for all three RioBridge
 * frames) and share it with whatever reads the encoders -- see {@code ModuleIOSpark} -- don't
 * construct a second one here.
 */
public class GyroIORioBridge implements GyroIO {
  private static final double STALE_THRESHOLD_SECONDS = 0.100;

  /**
   * Measured over-reading of this navX2's yaw: a hand-turned full revolution logged 396.31 degrees
   * against a true 360. Everything the navX reports about rotation is divided by this. See the
   * class javadoc -- this compensates for a faulty sensor and should go away if the IMU is fixed.
   */
  static final double YAW_SCALE = 396.31 / 360.0;

  private final RioBridgeCan bus;
  private Rotation2d lastRawYaw = Rotation2d.ZERO;
  private Rotation2d yawOffset = Rotation2d.ZERO;

  /** The navX's wrapped yaw from the previous frame, in degrees; NaN until the first frame. */
  private double previousRawYawDeg = Double.NaN;

  /** The navX's yaw with the +/-180 wraps unwound, in degrees. Scaling needs a continuous angle. */
  private double continuousRawYawDeg = 0.0;

  public GyroIORioBridge(RioBridgeCan bus) {
    this.bus = bus;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    bus.poll();

    AttitudeSample latest = bus.latestAttitude();
    inputs.connected =
        latest != null
            && (Timer.getMonotonicTimestamp() - latest.timestampSeconds()) < STALE_THRESHOLD_SECONDS;
    if (latest != null) {
      continuousRawYawDeg = unwrap(continuousRawYawDeg, previousRawYawDeg, latest.attitude().yawDeg());
      previousRawYawDeg = latest.attitude().yawDeg();
      lastRawYaw = yawFromNavx(continuousRawYawDeg);
      inputs.yawPosition = lastRawYaw.minus(yawOffset);
      inputs.yawVelocityRadPerSec = yawRateFromNavx(latest.attitude().yawRateDegPerSec());
    }
  }

  /**
   * Extends a running unwrapped angle by one new wrapped reading, in degrees. The first reading
   * (signalled by a NaN previous) starts the accumulator wherever the navX happens to be.
   */
  static double unwrap(double continuousDeg, double previousRawDeg, double rawDeg) {
    if (Double.isNaN(previousRawDeg)) {
      return rawDeg;
    }
    return continuousDeg + MathUtil.inputModulus(rawDeg - previousRawDeg, -180.0, 180.0);
  }

  /**
   * The navX's clockwise-positive, 10%-long yaw as a WPILib counter-clockwise-positive rotation.
   * Takes a continuous angle, not a wrapped one -- see the class javadoc for why that matters. The
   * offset in {@link #resetYaw()} is captured after this conversion, so both stay in WPILib's
   * convention.
   */
  static Rotation2d yawFromNavx(double navxContinuousYawDeg) {
    return Rotation2d.fromDegrees(-navxContinuousYawDeg / YAW_SCALE);
  }

  /** The navX's clockwise-positive yaw rate as counter-clockwise-positive radians per second. */
  static double yawRateFromNavx(double navxYawRateDegPerSec) {
    return Math.toRadians(-navxYawRateDegPerSec / YAW_SCALE);
  }

  @Override
  public void resetYaw() {
    // Can't reach the navX itself (see class javadoc) -- re-zero on this end instead by making
    // the next reported yawPosition relative to whatever the RioBridge is reporting right now.
    yawOffset = lastRawYaw;
  }
}
