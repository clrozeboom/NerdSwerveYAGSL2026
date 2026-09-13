// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Robot-wide constants.
 *
 * <p>Every drivetrain value here was carried over from the YAGSL {@code deploy/swerve/neo}
 * configuration and the previous {@code Constants.java}, so this project describes the same
 * physical robot the YAGSL project did. See the field comments for the handful of values that
 * looked unusual on the way across.
 */
public final class Constants {
  private Constants() {}

  /**
   * Enables the live-editable tuning entries and the bring-up routines on the dashboard.
   *
   * <p>Leave this true through bring-up: it is what lets gains be dragged on the dashboard between
   * step-response runs instead of redeployed. Set it false before competition, and every
   * {@link frc.robot.util.TunableNumber} collapses to its compiled-in value so nothing depends on a
   * dashboard entry that may not be set.
   */
  public static final boolean TUNING_MODE = false;

  /**
   * Parameters for the SysId routines, sized to the space available rather than to the defaults.
   *
   * <p>WPILib's stock SysId config (1 V/s ramp, 10 s timeout, 7 V step) assumes a normally-geared
   * drivetrain. This one is not: the inherited 1.36:1 reduction on a 2 in wheel implies a free speed
   * around 11 m/s, and at that speed the stock config would need roughly 43 m for a quasistatic test
   * and 34 m for a dynamic one. The whole field is 16 m.
   *
   * <p>The values below keep each test inside about 2.5 m. If the wheel-radius check shows the
   * drivetrain is actually geared down more than the config claims — which is the likely direction,
   * since 1.36:1 is very low for a swerve module — it will be slower than this and use even less
   * room, so these can be opened up once that is known.
   */
  public static final class SysId {
    private SysId() {}

    /** Quasistatic voltage ramp rate, in volts per second. */
    public static final double RAMP_RATE_VOLTS_PER_SEC = 0.5;

    /** Quasistatic cutoff, in seconds. Peak voltage is RAMP_RATE * QUASISTATIC_TIMEOUT. */
    public static final double QUASISTATIC_TIMEOUT_SECS = 4.0;

    /** Dynamic step voltage. */
    public static final double STEP_VOLTS = 2.0;

    /** Dynamic cutoff, in seconds. */
    public static final double DYNAMIC_TIMEOUT_SECS = 1.5;
  }

  /** Selects which {@code ModuleIO}/{@code GyroIO} implementations get wired up in RobotContainer. */
  public enum Mode {
    /** Running on a real SystemCore with real SPARK MAXes. */
    REAL,
    /** Running in desktop simulation. */
    SIM
  }

  public static final class Drivebase {
    private Drivebase() {}

    /**
     * Distance from robot center to each module, in inches, half the tread-center-to-tread-center
     * spacing. Measured on the robot: 10 in between the inner tread edges and 12 in across the
     * outer edges with 1 in treads, so the contact patches are 11 in apart and each sits 5.5 in
     * from center on both axes.
     *
     * <p>This replaces the 5.9375 the YAGSL module {@code location} blocks declared, which was
     * 7.4% too wide. It matters more than it looks: kinematics converts between wheel speeds and
     * chassis motion through this number, so an oversized track radius makes the robot rotate
     * faster than commanded and biases every wheel-radius estimate taken from a spin.
     *
     * <p>The tape says 5.5 (11 in between the tread centres, square on both axes). The robot says
     * 5.256, and that is the number used here. Rotating the robot through one full turn by hand,
     * with the modules parked tangentially so the wheels rolled freely, a rigid-body fit of the
     * four module displacements came back at 344.05 degrees where the truth was 360 -- the wheels
     * under-report rotation by 4.6%. Since rotation goes as distance over radius, under-reporting
     * means this radius is too big, and the straight-line test on {@link Module#WHEEL_RADIUS}
     * rules out the other two terms in that chain.
     *
     * <p>That fit is the best data in the drivetrain: per-module residuals of 0.001 to 0.05 in
     * across 45-49 in of roll, with no drive torque and so no slip. It survives uncertainty in how
     * exactly the turn was closed -- five degrees either way moves this to 5.19 or 5.33, never
     * back to 5.5.
     *
     * <p>Half an inch is still a lot to disagree with a tape about, and the discrepancy has no
     * mechanism yet. Worth measuring where the wheels actually touch the floor rather than where
     * the treads sit, and worth re-checking whether the wheels are offset from their steering
     * axes. Until then the measured-in-motion number wins, because it is the one the kinematics
     * actually has to be right about.
     */
    public static final double TRACK_RADIUS_X = Units.inchesToMeters(5.256);

    public static final double TRACK_RADIUS_Y = Units.inchesToMeters(5.256);

    /**
     * Module translations in the WPILib convention (+x forward, +y left), ordered front-left,
     * front-right, back-left, back-right. Every array in this project uses that same order.
     */
    public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_RADIUS_X, TRACK_RADIUS_Y),
      new Translation2d(TRACK_RADIUS_X, -TRACK_RADIUS_Y),
      new Translation2d(-TRACK_RADIUS_X, TRACK_RADIUS_Y),
      new Translation2d(-TRACK_RADIUS_X, -TRACK_RADIUS_Y)
    };

    /** Radius of the circle the modules sit on, used to convert max linear speed to max turn rate. */
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_RADIUS_X, TRACK_RADIUS_Y);

    /**
     * Default maximum linear speed, in m/s.
     *
     * <p>Carried over verbatim from the YAGSL project's {@code MAX_SPEED}, which was one foot per
     * second — a deliberate crawl, against the 4-5 m/s a real NEO swerve module is capable of. It
     * is the <i>default</i> rather than the limit: {@link
     * frc.robot.subsystems.drive.Drive#getMaxLinearSpeed()} wraps it in a tunable so the cap can be
     * raised from the dashboard during bring-up without a redeploy. Change this once a speed has
     * been settled on, so the robot boots with it.
     */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(2);

    /** Robot mass, from the previous Constants.java: (148 lb - 20.3 lb) converted to kg. */
    public static final double ROBOT_MASS_KG = (148 - 20.3) * 0.453592;

    /** How long the modules hold brake mode after being disabled, before coasting. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  public static final class Module {
    private Module() {}

    /**
     * Wheel radius, in inches. Calipered at 1.97-1.98 in diameter, so the nominal 2 in wheel the
     * YAGSL physicalproperties {@code drive.diameter} declared, worn very slightly under.
     *
     * <p>Confirmed in motion, not just with calipers: driving 82 in against a tape in teleop
     * logged 82.04 in of odometry, 0.05% long. Together with the bench-measured
     * {@link #DRIVE_GEAR_RATIO} that pins straight-line odometry, so any remaining disagreement
     * between the robot and its model is not in this number.
     *
     * <p>Pushing the robot 81 in while disabled logged only 78.7 in, 2.8% short, which is a
     * property of the test rather than of the wheel. At 1.35:1 the rotor is nearly directly
     * coupled to the wheel, so back-driving it has to turn a NEO against its own cogging; on a
     * slick floor the wheels skid instead of rolling and under-report. Prefer the driven test.
     */
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.975) / 2.0;

    /**
     * Drive reduction, measured rather than declared: ten hand turns of a blocked-up wheel reported
     * 62.369984 rad where the YAGSL physicalproperties {@code drive.gearRatio} of 1.36 predicts
     * 62.831853, putting the true reduction at 1.350003. Almost certainly 27:20 exactly.
     *
     * <p>This is a very low reduction for a swerve module — most are between 4:1 and 8:1 — and it
     * is real, not a config error. At 1.35 the free speed works out around 36 ft/s, which this
     * robot will never use; what it does mean is that the module is geared for speed and has very
     * little torque, and that driving at the 1 ft/s {@code MAX_LINEAR_SPEED} asks for roughly 0.69
     * V out of 12. Everything about this drivetrain's behaviour at that crawl — the stiction floor
     * {@link #DRIVE_KP} has to stay clear of above all — follows from operating a speed-geared
     * module at 6% output. Raising the speed cap would make the whole thing easier to control.
     *
     * <p>The correction from 1.36 also does not explain the spin residual it was meant to. With
     * wheel radius, track radius and now this all measured, a spin at a true 9.98 rad/s of wheel
     * speed should turn the robot at 1.268 rad/s; the gyro measured 1.404, still 10.8% fast. That
     * leaves the drive base radius and the navX's scale factor as the only unverified terms, and
     * one turn of the robot by hand separates them: 360 degrees of yaw means the gyro is honest
     * and the geometry is wrong, about 399 means the gyro over-reads.
     */
    public static final double DRIVE_GEAR_RATIO = 1.35;

    /** Turn reduction, from the YAGSL physicalproperties {@code angle.gearRatio}. */
    public static final double TURN_GEAR_RATIO = 19.127;

    /** Supply current limits, from the YAGSL physicalproperties {@code currentLimit} block. */
    public static final int DRIVE_CURRENT_LIMIT = 40;

    public static final int TURN_CURRENT_LIMIT = 20;

    /** Open-loop ramp rate in seconds, from the YAGSL {@code rampRate} block. */
    public static final double DRIVE_RAMP_RATE = 0.25;

    public static final double TURN_RAMP_RATE = 0.25;

    /**
     * Closed-loop ramp rate in seconds for the drive SPARK, seconds from neutral to full output.
     *
     * <p>The open-loop rate above does not apply to {@code ControlType.kVelocity}, so until this
     * was set nothing stopped the onboard velocity loop from slamming its output across the whole
     * range in a single 1 ms tick — in the spin step response the command swung 0.05 V to 1.55 V
     * and back at 2.8 Hz. Slew-limiting the output is the second half of the fix for that, next to
     * the reduced {@link #DRIVE_KP}: it bounds how fast the loop can dive below the static-friction
     * voltage and slam back over it.
     */
    public static final double DRIVE_CLOSED_LOOP_RAMP_RATE = 0.15;

    /** Nominal battery voltage, from the YAGSL {@code optimalVoltage}. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    /**
     * Whether the absolute encoders are actually readable this loop.
     *
     * <p>They still can't be wired to this SystemCore directly -- that hasn't changed. What
     * changed is that they no longer have to be: RioBridge (see {@link RioBridge}, below) runs
     * them on a roboRIO under their unmodified 2026 Thrifty vendor library and republishes the
     * readings over CAN, so {@code ModuleIOSpark} now reads its absolute position from {@code
     * RioBridgeCan} instead of a local {@code AnalogEncoder}. Set true here, matching that -- the
     * actual per-loop connected state {@code ModuleIOInputs.turnEncoderConnected} reports comes
     * from Encoders-frame staleness, not this constant, so a RioBridge power-cycle still shows up
     * as disconnected even with this true.
     *
     * <p>Set back to false (and revert {@code ModuleIOSpark} to a local {@code AnalogEncoder}) if
     * the RioBridge is ever removed from this robot -- without either one, a module has no idea
     * which way it is pointing at power-on, and needs the wheels aligned by hand and the turn
     * encoders zeroed via the "Zero Modules" routine instead.
     */
    public static final boolean HAS_ABSOLUTE_ENCODERS = true;

    /**
     * CAN bus the modules live on. SystemCore supports several, so REVLib 2027 requires a bus id
     * alongside the device id; 0 is the onboard bus. The YAGSL config left {@code canbus} null,
     * meaning the roboRIO's single bus, which maps to 0 here.
     */
    public static final int CAN_BUS_ID = 0;

    /**
     * Drive velocity gain, in <b>volts per wheel radian per second</b> of error.
     *
     * <p>The unit matters more than the number. Both IO layers take these gains in volts per unit
     * of error: {@code ModuleIOSim} feeds the controller output straight in as volts, and
     * {@code ModuleIOSpark} converts to the duty cycle a SPARK MAX closed loop actually wants. Give
     * a SPARK a volts-shaped gain directly and it is {@link #NOMINAL_VOLTAGE} times too aggressive.
     *
     * <p>The YAGSL carry-over here was 0.001, which is not a tune at all in these units — it gives
     * 0.012 V of authority at full speed, so the wheel neither reached its setpoint nor stopped
     * when asked. 0.2 replaced it, sized in simulation, where it reaches 95% of a step in 1.08 s
     * with no overshoot.
     *
     * <p>On hardware 0.2 turned the loop into a bang-bang oscillator. Simulation has no static
     * friction to fall below; the real drive does. At a 10 rad/s setpoint the feedforward sits at
     * {@link #DRIVE_KS} + {@link #DRIVE_KV} * 10 = 0.690 V, only 0.27 V above the 0.423 V it takes
     * to break the wheel loose at all. At kP 0.2 that margin is spent by 1.33 rad/s of overspeed,
     * so the loop kept commanding itself below breakaway: in the spin step response the command sat
     * under kS 40% of the time, the wheel coasted down, the command slammed back over kS, and the
     * whole thing limit-cycled at 2.8 Hz with the wheel surging between 2 and 23 rad/s against a
     * 10 rad/s setpoint. The mean looked perfect throughout — the SPARK's filtered velocity signal
     * hides most of the amplitude, and it only shows up in raw encoder position.
     *
     * <p>0.02 keeps the command above kS through +/-13.3 rad/s of error, which covers the observed
     * swing. The feedforward was already carrying the load unaided (measured mean applied voltage
     * matched the feedforward-alone prediction), so there is little for proportional to do here;
     * its job is trimming, not driving. Raise it from the dashboard if you want, but the ceiling is
     * the stiction floor above, not stability.
     */
    public static final double DRIVE_KP = 0.02;

    public static final double DRIVE_KD = 0.0;

    /**
     * Drive feedforward, measured with SysId on the YAGSL project and averaged across all four
     * modules. Unlike the PID gains above these are physical and do carry over, but they were
     * measured in <b>volts per m/s</b> of wheel surface speed, so {@link #DRIVE_KV} and
     * {@link #DRIVE_KA} below convert them into the volts-per-wheel-rad/s this project's IO layer
     * works in.
     *
     * <p>Both measured by {@code TuningCommands.steadyStateSweep}, which holds each voltage until
     * the speed settles and so carries no acceleration term at all. Ten steps from 0.15 V to
     * 1.05 V fit a straight line to r-squared 0.9994 or better on every module, with kS between
     * 0.1969 and 0.2032 and kV between 0.02592 and 0.02661 V per wheel rad/s. The spread is small
     * enough to be scatter, so both are shared rather than held per corner.
     *
     * <p>The history is worth keeping, because it is a lesson about the measurement rather than
     * about the robot. The inherited kS was 0.4234. A voltage ramp fit 0.3501 and the SysId
     * quasistatic sweep fit 0.3184, which put it at 0.334 -- still 67% high. Every one of those
     * routines ramps voltage at a constant rate, which on a linear plant means constant
     * acceleration for the whole run, which makes kS and kA collinear: a three-parameter fit
     * returns kA near zero and leaves the entire acceleration contribution sitting in the
     * intercept. Each successive fix cut the drive's overspeed on a low-friction floor -- 42% at
     * 0.4234, 25% at 0.334 -- without ever reaching the cause, because the cause was that a ramp
     * cannot measure a static quantity.
     *
     * <p>That difference also puts a number on kA, which nothing reads today but which matters
     * the moment this drivetrain follows a path: 0.3501 - 0.2005 over the ~16 rad/s^2 those ramps
     * held is about 0.0092 V per wheel rad/s^2, near 0.37 V/(m/s^2), against the 0.129 that
     * {@link #DRIVE_KA_PER_METER_PER_SEC2} inherited. Roughly three times. Measure it properly
     * before trusting it.
     *
     * <p>The old per-corner note is refuted. It had front-right's static friction at ~0.65 V
     * against ~0.31-0.41 for the others; front-right is mid-pack in all three runs since. (The
     * turn side is the opposite case: there the per-module differences did reproduce, and it
     * holds per-corner kS.)
     *
     * <p>Breakaway -- the voltage that first moves a stopped wheel -- came in at 0.25 V on all
     * four, against a 0.15 V step that moved nothing. Note how close that is to the 0.2005
     * intercept, and that the 0.25 V point sits exactly on the fitted line: this drivetrain has
     * far less stiction than the inherited gains implied. The step was 0.1 V, so breakaway is
     * only known to lie in (0.15, 0.25]; sweep more finely down there if it ever matters.
     */
    public static final double DRIVE_KS = 0.2005;

    public static final double DRIVE_KV_PER_METER_PER_SEC = 1.0529;

    public static final double DRIVE_KA_PER_METER_PER_SEC2 = 0.129;

    /** {@link #DRIVE_KV_PER_METER_PER_SEC} expressed in volts per wheel radian per second. */
    public static final double DRIVE_KV = DRIVE_KV_PER_METER_PER_SEC * WHEEL_RADIUS;

    /** {@link #DRIVE_KA_PER_METER_PER_SEC2} expressed in volts per wheel radian per second squared. */
    public static final double DRIVE_KA = DRIVE_KA_PER_METER_PER_SEC2 * WHEEL_RADIUS;

    /**
     * Turn position gains, carried over from the YAGSL pidfproperties {@code angle} block. The same
     * unit caveat as {@link #DRIVE_KP} applies — volts per radian of error, and this loop runs in
     * module radians.
     *
     * <p><b>There is a hard ceiling on this gain, and it is lower than the value the parked error
     * would suggest.</b> The turn loop closes over the RioBridge at the robot's 50 Hz, and that
     * feedback has a stale tail — median 20 ms but p99 around 160 ms. A proportional loop driving a
     * velocity plant goes unstable once gain × plant gain × delay exceeds about π/2, which for this
     * drivetrain (~48 deg/s per volt, measured) puts the limit near kP 12 at the p99 delay. kP 8 was
     * tried on 2026-09-09 and the modules span continuously through full revolutions rather than
     * settling — 941 degrees of travel per second against 167 at kP 2.8.
     *
     * <p>Since the gain needed to overcome stiction by proportional action alone is roughly 12 for
     * the stiffest corner, the two constraints do not overlap. That is what
     * {@link ModuleConfig#turnKs} is for: it defeats friction without raising loop gain. Do not
     * raise this to chase steady-state error.
     *
     * <p>2.1 is tuned, not assumed. It was chosen against 2.5 on hardware over two sessions: both
     * park accurately, but 2.5 overshoots roughly 34 degrees on the way to a setpoint against about
     * 2 for 2.1. On a swerve module that overshoot is wheel scrub and a lurch, so the slower
     * approach is worth the marginally larger parked error. A gain change that small mattering that
     * much is itself a reminder of how close the delay ceiling is.
     *
     * <p>The YAGSL carry-over was 0.01, which left a commanded 90 degree module turn sitting at
     * 6.8 degrees three seconds later; the modules effectively did not steer. 2.0 completes the
     * same step in 0.52 s with no overshoot in simulation.
     */
    public static final double TURN_KP = 2.1;

    public static final double TURN_KD = 0.0;

    /**
     * How close a module has to be before the turn feedforward switches off, in degrees.
     *
     * <p>{@link ModuleConfig#turnKs} pushes at a fixed voltage whenever the module is outside this
     * band, which is what defeats stiction. Inside it the term is dropped entirely: a fixed push
     * that never switches off would drive straight past the setpoint, get pushed back, and hunt
     * forever. This band is therefore the accuracy the loop can settle to -- there is no point
     * setting it tighter than the absolute encoder can actually resolve.
     *
     * <p>It is also what dominates how well the robot drives a closed path, which is why it sits
     * at 0.75 rather than the 1.5 it was tuned to. A module can park anywhere inside the band and
     * stay there, and it settles differently depending on which side it arrived from -- at 1.5 the
     * modules averaged 0.58 degrees off driving the square's forward leg against 3.04 degrees off
     * on the leg back, so opposing legs were not quite antiparallel. Halving the band closed that
     * worst asymmetry to 0.36 degrees and took the square's closure from 1.23 in to 0.125 in over
     * the same 118 in of path. Odometry's own error against the floor went from 0.68 in to 0.25 in
     * alongside it.
     *
     * <p>Ten times better for half the band is a much bigger win than the mechanism above predicts,
     * and no attempt to reconstruct the closure from the logged module headings has held up across
     * both runs -- one such sum matched the 1.5 result and then missed the 0.75 result by an inch,
     * and it disagrees with the odometry pose it should reproduce. So take the improvement as
     * measured and the explanation as incomplete: the pointing asymmetry is real and moves the
     * right way, but something else is contributing and has not been identified.
     *
     * <p>The band cannot shrink indefinitely. It exists because a fixed push that never switches
     * off hunts forever, and the feedforward is what took parked pointing error from 11.9 degrees
     * to 1.07. At 0.75 there is no sign of hunting -- module heading stays within about 1 degree
     * rms of its own mean on a leg, the same as at 1.5 -- but that margin is now thin, and the
     * absolute encoder's own resolution is the floor underneath it. Re-run the turn step response
     * as well as the square before going lower.
     */
    public static final double TURN_FEEDFORWARD_TOLERANCE_DEG = 0.75;

    /** Simulated rotational inertia. Not a YAGSL value — only used by ModuleIOSim. */
    public static final double DRIVE_SIM_MOI = 0.025;

    public static final double TURN_SIM_MOI = 0.004;
  }

  /**
   * Per-corner hardware configuration. CAN IDs, analog encoder channels and absolute encoder offsets
   * are exactly the values from the four YAGSL module JSONs.
   */
  public enum ModuleConfig {
    FRONT_LEFT(1, 2, 0, 163.48, Module.DRIVE_KS, Module.DRIVE_KV, 0.43),
    FRONT_RIGHT(7, 8, 1, 338.55, Module.DRIVE_KS, Module.DRIVE_KV, 0.15),
    BACK_LEFT(3, 4, 2, 9.32, Module.DRIVE_KS, Module.DRIVE_KV, 0.30),
    BACK_RIGHT(5, 6, 3, 283.62, Module.DRIVE_KS, Module.DRIVE_KV, 0.27);

    /** SPARK MAX CAN ID driving the wheel. */
    public final int driveCanId;

    /** SPARK MAX CAN ID steering the module. */
    public final int turnCanId;

    /** Analog input channel for the Thrifty absolute encoder. */
    public final int encoderChannel;

    /** Absolute encoder reading, in degrees, when the module points straight forward. */
    public final double absoluteEncoderOffsetDegrees;

    /**
     * This corner's static friction feedforward, in volts.
     *
     * <p>Held per module because static friction genuinely differs corner to corner — bearing
     * preload, seal drag and gear mesh are not identical across four hand-built modules. The 2026
     * project's SysId run measured front-right at roughly 0.65 V against 0.31-0.41 V on the other
     * three, which is a large enough spread to be worth carrying separately rather than averaging
     * away. YAGSL could not express that; this can.
     *
     * <p>All four still default to the averaged {@link Module#DRIVE_KS}, because the old measurement
     * recorded which corner was the outlier but not which value belonged to each of the other three.
     * Run the feedforward ramp and fill in the real numbers.
     */
    public final double driveKs;

    /**
     * This corner's velocity feedforward, in volts per wheel radian per second.
     *
     * <p>Also per module, though expect much less spread than {@link #driveKs}: kV is set by gearing
     * and motor constants, which are the same part in all four corners. A corner that comes out
     * noticeably different here is more likely to be a mechanical problem than a gain worth keeping.
     */
    public final double driveKv;

    /**
     * Voltage this corner's steering needs before it will move at all, in volts.
     *
     * <p>Measured directly, not guessed: with only proportional control each module coasts to a
     * stop where the voltage its remaining error produces falls below its own breakaway, and sits
     * there holding exactly that voltage. Reading the held voltage out of a turn step response log
     * therefore measures the friction. These four came from the 2026-09-09 runs, and the spread is
     * real — front-left needs nearly three times front-right, which is worth a look at the module
     * itself rather than only compensating for in software.
     */
    public final double turnKs;

    ModuleConfig(
        int driveCanId,
        int turnCanId,
        int encoderChannel,
        double offsetDegrees,
        double driveKs,
        double driveKv,
        double turnKs) {
      this.driveCanId = driveCanId;
      this.turnCanId = turnCanId;
      this.encoderChannel = encoderChannel;
      this.absoluteEncoderOffsetDegrees = offsetDegrees;
      this.driveKs = driveKs;
      this.driveKv = driveKv;
      this.turnKs = turnKs;
    }

    /** All four corners in the canonical FL, FR, BL, BR order. */
    public static final ModuleConfig[] ORDERED = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
  }

  public static final class Operator {
    private Operator() {}

    /** Joystick deadband, from the previous OperatorConstants. */
    public static final double DEADBAND = 0.1;
  }

  /**
   * Wiring for the <a href="https://github.com/clrozeboom/RioBridge">RioBridge</a> -- a roboRIO
   * that republishes the four Thrifty absolute encoders and a navX2 over CAN, since neither can
   * be wired to this SystemCore directly. See that repo's README for the protocol and its
   * hardware-verification guide for how to check this bench setup before trusting it.
   */
  public static final class RioBridge {
    private RioBridge() {}

    /**
     * The RioBridge's dedicated CAN bus, isolated from {@link Module#CAN_BUS_ID} so its frames
     * never mix with the drivetrain's SPARK MAX / heartbeat traffic. A raw HAL bus id ({@code
     * int}), not {@code CANPort} -- see {@code RioBridgeCan}'s javadoc for why, on this project's
     * alpha-6 WPILib pin.
     */
    public static final int BUS_ID = CANBusMap.CAN_S1;
  }
}
