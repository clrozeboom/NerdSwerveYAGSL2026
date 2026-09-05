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
     * Default maximum linear speed, in m/s.
     *
     * <p>Carried over verbatim from the YAGSL project's {@code MAX_SPEED}, which was one foot per
     * second — a deliberate crawl, against the 4-5 m/s a real NEO swerve module is capable of. It
     * is the <i>default</i> rather than the limit: {@link
     * frc.robot.subsystems.drive.Drive#getMaxLinearSpeed()} wraps it in a tunable so the cap can be
     * raised from the dashboard during bring-up without a redeploy. Change this once a speed has
     * been settled on, so the robot boots with it.
     */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(1);

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
     * when asked. 0.2 reaches 95% of a step in 1.08 s with no overshoot in simulation. Simulation
     * has none of the latency or backlash real hardware does, so treat this as a starting point
     * with margin and raise it from the dashboard.
     */
    public static final double DRIVE_KP = 0.2;

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
     * unit caveat as {@link #DRIVE_KP} applies — volts per radian of error, and this loop runs in
     * module radians.
     *
     * <p>The YAGSL carry-over was 0.01, which left a commanded 90 degree module turn sitting at
     * 6.8 degrees three seconds later; the modules effectively did not steer. 2.0 completes the
     * same step in 0.52 s with no overshoot in simulation.
     */
    public static final double TURN_KP = 2.0;

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
