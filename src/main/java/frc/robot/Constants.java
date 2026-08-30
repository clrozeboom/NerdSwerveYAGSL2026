// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final boolean TUNING_MODE = true;

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
     * Distance from robot center to each module, from the YAGSL module {@code location} blocks
     * (front/left, in inches). All four sit 5.9375 in from center on both axes.
     */
    public static final double TRACK_RADIUS_X = Units.inchesToMeters(5.9375);

    public static final double TRACK_RADIUS_Y = Units.inchesToMeters(5.9375);

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
     * Maximum linear speed. Carried over verbatim from the YAGSL project's {@code MAX_SPEED}, which
     * was {@code Units.feetToMeters(1)} — about 0.3 m/s. That is a deliberately crawling value; a
     * real NEO swerve module is capable of roughly 4-5 m/s. Raise this once the drivetrain is
     * trusted, and note that MAX_ANGULAR_SPEED below scales with it.
     */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(1);

    /** Maximum turn rate, derived from the linear speed and the drive base radius. */
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    /** Robot mass, from the previous Constants.java: (148 lb - 20.3 lb) converted to kg. */
    public static final double ROBOT_MASS_KG = (148 - 20.3) * 0.453592;

    /** How long the modules hold brake mode after being disabled, before coasting. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  public static final class Module {
    private Module() {}

    /** Wheel diameter, from the YAGSL physicalproperties {@code drive.diameter} (inches). */
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0) / 2.0;

    /**
     * Drive reduction, from the YAGSL physicalproperties {@code drive.gearRatio}. 1.36 is a very low
     * reduction for a swerve module — most are between 4:1 and 8:1 — but it is what the YAGSL
     * config declared, so it is carried across unchanged. Worth confirming against the physical
     * module before driving at speed.
     */
    public static final double DRIVE_GEAR_RATIO = 1.36;

    /** Turn reduction, from the YAGSL physicalproperties {@code angle.gearRatio}. */
    public static final double TURN_GEAR_RATIO = 19.127;

    /** Supply current limits, from the YAGSL physicalproperties {@code currentLimit} block. */
    public static final int DRIVE_CURRENT_LIMIT = 40;

    public static final int TURN_CURRENT_LIMIT = 20;

    /** Open-loop ramp rate in seconds, from the YAGSL {@code rampRate} block. */
    public static final double DRIVE_RAMP_RATE = 0.25;

    public static final double TURN_RAMP_RATE = 0.25;

    /** Nominal battery voltage, from the YAGSL {@code optimalVoltage}. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    /**
     * Whether the Thrifty absolute encoders are actually wired up.
     *
     * <p>Currently false: they cannot be connected to this SystemCore. Without them a module has no
     * idea which way it is pointing at power-on, so instead the wheels are aligned by hand and the
     * turn encoders zeroed there — either at boot or on demand with the "Zero Modules" routine.
     *
     * <p>What that costs: module heading is only correct as long as nothing moves the wheels between
     * zeroing and driving. Bump a module, or let the steering coast while pushing the robot around,
     * and it is wrong until you re-zero. Re-zero whenever the robot has been handled, and expect to
     * do it more often than feels reasonable.
     *
     * <p>Set true once the encoders are connected, and the modules will seed themselves from the
     * absolute reading at boot instead.
     */
    public static final boolean HAS_ABSOLUTE_ENCODERS = false;

    /**
     * CAN bus the modules live on. SystemCore supports several, so REVLib 2027 requires a bus id
     * alongside the device id; 0 is the onboard bus. The YAGSL config left {@code canbus} null,
     * meaning the roboRIO's single bus, which maps to 0 here.
     */
    public static final int CAN_BUS_ID = 0;

    /**
     * Drive velocity gains, carried over from the YAGSL pidfproperties {@code drive} block.
     *
     * <p>Treat these as a starting point, not a tune. PID gains only mean anything relative to the
     * units their controller works in, and YAGSL ran its drive loop on the SPARK MAX in motor
     * rotations while this project closes the loop in wheel radians per second. The number is here
     * so nothing is silently invented; expect to re-tune it on the real robot.
     */
    public static final double DRIVE_KP = 0.001;

    public static final double DRIVE_KD = 0.0;

    /**
     * Drive feedforward, measured with SysId on the YAGSL project and averaged across all four
     * modules. Unlike the PID gains above these are physical and do carry over, but they were
     * measured in <b>volts per m/s</b> of wheel surface speed, so {@link #DRIVE_KV} and
     * {@link #DRIVE_KA} below convert them into the volts-per-wheel-rad/s this project's IO layer
     * works in.
     *
     * <p>The previous Constants.java noted that front-right's static friction measured noticeably
     * higher than the other three (~0.65 V against ~0.31-0.41 V) and was worth a physical look.
     * That note still stands, and unlike YAGSL this project could hold per-module gains if you
     * decide to chase it.
     */
    public static final double DRIVE_KS = 0.4234;

    public static final double DRIVE_KV_PER_METER_PER_SEC = 1.0618;

    public static final double DRIVE_KA_PER_METER_PER_SEC2 = 0.129;

    /** {@link #DRIVE_KV_PER_METER_PER_SEC} expressed in volts per wheel radian per second. */
    public static final double DRIVE_KV = DRIVE_KV_PER_METER_PER_SEC * WHEEL_RADIUS;

    /** {@link #DRIVE_KA_PER_METER_PER_SEC2} expressed in volts per wheel radian per second squared. */
    public static final double DRIVE_KA = DRIVE_KA_PER_METER_PER_SEC2 * WHEEL_RADIUS;

    /**
     * Turn position gains, carried over from the YAGSL pidfproperties {@code angle} block. The same
     * unit caveat as {@link #DRIVE_KP} applies — this loop runs in module radians here.
     */
    public static final double TURN_KP = 0.01;

    public static final double TURN_KD = 0.0;

    /** Simulated rotational inertia. Not a YAGSL value — only used by ModuleIOSim. */
    public static final double DRIVE_SIM_MOI = 0.025;

    public static final double TURN_SIM_MOI = 0.004;
  }

  /**
   * Per-corner hardware configuration. CAN IDs, analog encoder channels and absolute encoder offsets
   * are exactly the values from the four YAGSL module JSONs.
   */
  public enum ModuleConfig {
    FRONT_LEFT(1, 2, 0, 169.5, Module.DRIVE_KS, Module.DRIVE_KV),
    FRONT_RIGHT(7, 8, 1, 342.5, Module.DRIVE_KS, Module.DRIVE_KV),
    BACK_LEFT(3, 4, 2, 14.99, Module.DRIVE_KS, Module.DRIVE_KV),
    BACK_RIGHT(5, 6, 3, 290.4, Module.DRIVE_KS, Module.DRIVE_KV);

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

    ModuleConfig(
        int driveCanId,
        int turnCanId,
        int encoderChannel,
        double offsetDegrees,
        double driveKs,
        double driveKv) {
      this.driveCanId = driveCanId;
      this.turnCanId = turnCanId;
      this.encoderChannel = encoderChannel;
      this.absoluteEncoderOffsetDegrees = offsetDegrees;
      this.driveKs = driveKs;
      this.driveKv = driveKv;
    }

    /** All four corners in the canonical FL, FR, BL, BR order. */
    public static final ModuleConfig[] ORDERED = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
  }

  public static final class Operator {
    private Operator() {}

    /** Joystick deadband, from the previous OperatorConstants. */
    public static final double DEADBAND = 0.1;
  }
}
