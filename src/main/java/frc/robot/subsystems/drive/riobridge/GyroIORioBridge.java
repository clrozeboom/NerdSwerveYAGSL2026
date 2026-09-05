package frc.robot.subsystems.drive.riobridge;

import frc.robot.subsystems.drive.GyroIO;
import org.wpilib.math.geometry.Rotation2d;
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
 * history. {@link #updateInputs} still drains {@link RioBridgeCan#drainAttitudeSamples()} every
 * loop regardless, just to discard the extras -- {@code RioBridgeCanDemux} buffers every sample
 * since the last drain, and nothing here would ever empty that buffer otherwise.
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
 * <p>Construct one {@link RioBridgeCan} per robot (it owns the one CAN stream session for all
 * three RioBridge frames) and share it with whatever reads the encoders -- see {@code
 * ModuleIOSpark} -- don't open a second session here.
 */
public class GyroIORioBridge implements GyroIO {
  private static final double STALE_THRESHOLD_SECONDS = 0.100;

  private final RioBridgeCan bus;
  private Rotation2d lastRawYaw = Rotation2d.kZero;
  private Rotation2d yawOffset = Rotation2d.kZero;

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
      lastRawYaw = Rotation2d.fromDegrees(latest.attitude().yawDeg());
      inputs.yawPosition = lastRawYaw.minus(yawOffset);
      inputs.yawVelocityRadPerSec = Math.toRadians(latest.attitude().yawRateDegPerSec());
    }

    // This GyroIO has nowhere to put per-sample odometry data (see class javadoc), but something
    // still has to drain the buffer each loop or RioBridgeCanDemux's pending-samples list grows
    // without bound.
    bus.drainAttitudeSamples();
  }

  @Override
  public void resetYaw() {
    // Can't reach the navX itself (see class javadoc) -- re-zero on this end instead by making
    // the next reported yawPosition relative to whatever the RioBridge is reporting right now.
    yawOffset = lastRawYaw;
  }
}
