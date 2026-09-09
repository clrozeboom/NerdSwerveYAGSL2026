// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConfig;
import frc.robot.protocol.CanFrames.EncodersFrame;
import frc.robot.subsystems.drive.riobridge.RioBridgeCan;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

/**
 * Real-hardware module: two SPARK MAX / NEO pairs plus a Thrifty absolute encoder, matching the
 * hardware described by the YAGSL {@code deploy/swerve/neo} config.
 *
 * <p>Closed-loop control runs on the SPARK MAX itself, which is why this class configures gains onto
 * the controller rather than owning PID objects the way {@link ModuleIOSim} does.
 *
 * <p>REVLib 2027 reshaped its read API: every getter now returns a {@link Signal}, which carries the
 * value together with a timestamp and a validity flag rather than a bare double. That is what feeds
 * the {@code *Connected} inputs below — a stale or errored signal is exactly the "controller did not
 * answer this cycle" case the IO layer wants to report. Commanding a closed-loop setpoint is
 * {@code setSetpoint} here, not the {@code setReference} of previous seasons.
 *
 * <p>The Thrifty encoder itself can't be wired to this SystemCore directly ({@link
 * Constants.Module#HAS_ABSOLUTE_ENCODERS}'s javadoc has why), so its reading arrives over CAN
 * through the shared {@link RioBridgeCan} instead of a local {@code AnalogEncoder}. That instance
 * only actually gets polled once per loop from {@code GyroIORioBridge.updateInputs()} (see {@code
 * Drive.periodic()}'s ordering, gyro before modules) -- if this robot ever drops back to a gyro
 * that isn't RioBridge-backed while leaving this true, nothing will poll the shared session and
 * every module's absolute reading will go stale silently.
 */
public class ModuleIOSpark implements ModuleIO {
  /** RioBridge Encoders-frame staleness threshold, matching {@code GyroIORioBridge}'s. */
  private static final double ENCODER_STALE_THRESHOLD_SECONDS = 0.100;

  /** {@code EncodersFrame.rawCounts()} is a 12-bit ADC, i.e. 4096 possible codes. */
  private static final double ENCODER_ADC_COUNTS = 4096.0;

  /** How long to block at construction waiting for the first Encoders frame; see below. */
  private static final double ENCODER_SEED_TIMEOUT_SECONDS = 0.5;
  /** Wheel radians per motor rotation. */
  private static final double DRIVE_POSITION_FACTOR =
      (2 * Math.PI) / Constants.Module.DRIVE_GEAR_RATIO;

  /** Wheel radians per second per motor RPM. */
  private static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0;

  /** Module radians per motor rotation. */
  private static final double TURN_POSITION_FACTOR =
      (2 * Math.PI) / Constants.Module.TURN_GEAR_RATIO;

  /** Module radians per second per motor RPM. */
  private static final double TURN_VELOCITY_FACTOR = TURN_POSITION_FACTOR / 60.0;

  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  /**
   * The turn loop, closed here on the absolute encoder rather than on the SPARK MAX's onboard
   * controller.
   *
   * <p>The SPARK's own loop can only see the turn motor's encoder, and this robot has 7-10 degrees
   * of backlash between that motor and the module. Closing on the motor puts the *motor* where it
   * was asked and leaves the module anywhere inside the lash band; closing on the absolute encoder
   * puts the module where it was asked and lets the motor take up whatever lash it needs.
   *
   * <p>The cost is loop rate and latency: this runs at the robot loop's 50 Hz against the SPARK's
   * internal kHz, and the measurement arrives over CAN from the RioBridge (median 20 ms, but with
   * a tail out past 150 ms). Steering is slow enough for that to be fine — a module slews 157
   * degrees in 0.4 s — but it is why there is no derivative term by default: differentiating a
   * signal that sometimes repeats for several loops produces spikes, not damping.
   *
   * <p>A side effect worth having: {@code ModuleIOSim} already closes its turn loop exactly this
   * way, so {@code TURN_KP} now means the same thing in simulation and on the robot, in volts per
   * radian of error. It no longer goes through {@link #voltsPerErrorToDuty}, which applies only to
   * gains handed to a SPARK's onboard loop -- the drive loop still is one.
   */
  private final PIDController turnController =
      new PIDController(Constants.Module.TURN_KP, 0.0, Constants.Module.TURN_KD);

  /** Heading the turn loop is driving towards, or null when nothing has commanded one yet. */
  private Rotation2d turnSetpoint = null;

  /** This corner's measured steering breakaway voltage; see {@link ModuleConfig#turnKs}. */
  private double turnKs;

  /** Half-width of the band inside which the feedforward is dropped, in radians. */
  private double turnFeedforwardToleranceRad =
      Math.toRadians(Constants.Module.TURN_FEEDFORWARD_TOLERANCE_DEG);
  private final RioBridgeCan rioBridgeCan;
  private final int encoderChannel;
  private final Rotation2d absoluteEncoderOffset;

  private double driveKs = Constants.Module.DRIVE_KS;
  private double driveKv = Constants.Module.DRIVE_KV;

  /**
   * Converts a gain expressed in volts per unit of error into the duty cycle a SPARK MAX closed
   * loop wants.
   *
   * <p>Applies to the <b>drive</b> loop only, which is the one still running on a SPARK. The turn
   * loop moved into this class (see {@link #turnController}) and takes its gains in volts directly,
   * so it does not go through here.
   *
   * <p>This project states its PID gains in volts per unit of error, because that is what
   * {@code ModuleIOSim} applies and what makes a gain comparable between the two IO layers. A SPARK
   * MAX closed loop does not work in volts: its output is a duty cycle in [-1, 1], which
   * {@link Constants.Module#NOMINAL_VOLTAGE} voltage compensation then maps onto that many volts.
   * Passing a volts-shaped gain straight through would therefore be {@code NOMINAL_VOLTAGE} times
   * too aggressive — a kP tuned to a well-behaved step in simulation would saturate the controller
   * on the robot.
   *
   * <p>The arbitrary feedforward is <i>not</i> converted: REVLib takes that in volts already
   * (the four-argument {@code setSetpoint} defaults to {@code ArbFFUnits.kVoltage}), which is why
   * {@link #setDriveVelocity(double)} passes it through untouched.
   */
  static double voltsPerErrorToDuty(double gain) {
    return gain / Constants.Module.NOMINAL_VOLTAGE;
  }

  /**
   * @param rioBridgeCan the robot's one shared RioBridge session (see {@code RobotContainer}), or
   *     {@code null} if this robot has no RioBridge -- only read when {@link
   *     Constants.Module#HAS_ABSOLUTE_ENCODERS} is true, same as the {@code AnalogEncoder} this
   *     replaced only got constructed when that flag was true.
   */
  public ModuleIOSpark(ModuleConfig config, RioBridgeCan rioBridgeCan) {
    driveSpark = new SparkMax(Constants.Module.CAN_BUS_ID, config.driveCanId, MotorType.kBrushless);
    turnSpark = new SparkMax(Constants.Module.CAN_BUS_ID, config.turnCanId, MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();

    // The module wraps, so let the controller take the short way round rather than unwinding.
    // This replaces the SPARK's positionWrapping config, which only applied to its onboard loop.
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    this.rioBridgeCan = Constants.Module.HAS_ABSOLUTE_ENCODERS ? rioBridgeCan : null;
    this.encoderChannel = config.encoderChannel;
    this.turnKs = config.turnKs;
    absoluteEncoderOffset = Rotation2d.fromDegrees(config.absoluteEncoderOffsetDegrees);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Module.DRIVE_CURRENT_LIMIT)
        .voltageCompensation(Constants.Module.NOMINAL_VOLTAGE)
        .openLoopRampRate(Constants.Module.DRIVE_RAMP_RATE);
    driveConfig.encoder.positionConversionFactor(DRIVE_POSITION_FACTOR);
    driveConfig.encoder.velocityConversionFactor(DRIVE_VELOCITY_FACTOR);
    driveConfig.closedLoop.pid(
        voltsPerErrorToDuty(Constants.Module.DRIVE_KP),
        0.0,
        voltsPerErrorToDuty(Constants.Module.DRIVE_KD));
    driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Module.TURN_CURRENT_LIMIT)
        .voltageCompensation(Constants.Module.NOMINAL_VOLTAGE)
        .openLoopRampRate(Constants.Module.TURN_RAMP_RATE);
    turnConfig.encoder.positionConversionFactor(TURN_POSITION_FACTOR);
    turnConfig.encoder.velocityConversionFactor(TURN_VELOCITY_FACTOR);
    // No closed-loop config for the turn SPARK any more: the position loop lives here now, on the
    // absolute encoder, and this controller is driven open-loop with the voltage it asks for. The
    // motor encoder is still configured because it is worth logging for backlash.
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // With an absolute encoder the module can work out where it is pointing on its own. Without one
    // it has to be told, so assume it starts aligned — whoever powered the robot on is expected to
    // have straightened the wheels, and the "Zero Modules" routine re-asserts it on demand.
    if (rioBridgeCan != null) {
      awaitFirstEncodersFrame();
    }
    turnEncoder.setPosition(
        Constants.Module.HAS_ABSOLUTE_ENCODERS ? readAbsolutePosition().getRadians() : 0.0);
    driveEncoder.setPosition(0.0);
  }

  /**
   * Blocks briefly for the RioBridge's first Encoders frame so the seed above reads a real
   * absolute position rather than falling into {@link #readAbsolutePosition()}'s "nothing's
   * arrived yet" path -- {@code RioBridgeCan.poll()} otherwise only ever gets called from the
   * periodic loop, well after this constructor has already returned. Since {@code rioBridgeCan}
   * is one instance shared across all four modules, only the first one built actually waits; by
   * the time the rest construct, {@code latestEncoders()} is already populated.
   */
  private void awaitFirstEncodersFrame() {
    double deadline = Timer.getMonotonicTimestamp() + ENCODER_SEED_TIMEOUT_SECONDS;
    while (rioBridgeCan.latestEncoders() == null && Timer.getMonotonicTimestamp() < deadline) {
      rioBridgeCan.poll();
    }
    if (rioBridgeCan.latestEncoders() == null) {
      System.out.println(
          "ModuleIOSpark: no RioBridge Encoders frame within "
              + ENCODER_SEED_TIMEOUT_SECONDS
              + "s at boot -- seeding this module as if aligned. Check the RioBridge is powered"
              + " and transmitting, then re-zero with the \"Zero Modules\" routine.");
    }
  }

  /**
   * Drives the turn motor towards {@link #turnSetpoint} using the module's own measured heading.
   *
   * <p>Refuses to act on a stale reading. The RioBridge delivers encoders at a median 20 ms but
   * with a tail past 150 ms, and a position loop fed a frozen measurement keeps pushing on an error
   * it can no longer see shrinking -- which on a steering motor means winding harder into the
   * gearbox until the frame finally arrives. Coasting until it does is the safe failure: the module
   * stops where it is instead of being driven blind.
   *
   * <p>Falls back to the turn motor's own encoder when the absolute reading goes stale, rather than
   * stopping. Steering on the motor is what this code did before the backlash was measured: it is
   * wrong by however much lash is taken up, but a module that steers to within 10 degrees is worth
   * a great deal more mid-match than one that has stopped steering. The two encoders share a frame
   * -- the motor's is seeded from the absolute at boot -- so the same setpoint means the same thing
   * to both, and switching between them costs a step in the output and nothing else, there being no
   * integral term to wind up.
   *
   * @param measured the module heading read this cycle
   * @param encoderFresh whether that reading is recent enough to steer on
   * @param motorMeasured the turn motor's own idea of the heading, used when it is not
   */
  private void runTurnControl(Rotation2d measured, boolean encoderFresh, Rotation2d motorMeasured) {
    if (turnSetpoint == null) {
      turnController.reset();
      turnSpark.setVoltage(0.0);
      return;
    }
    Rotation2d feedback = encoderFresh ? measured : motorMeasured;
    double volts = turnController.calculate(feedback.getRadians(), turnSetpoint.getRadians());
    // getError() is the wrapped error the controller just used, so this picks the same short way
    // round the circle that the proportional term did.
    volts += turnFeedforwardVolts(turnController.getError(), turnKs, turnFeedforwardToleranceRad);
    turnSpark.setVoltage(
        Math.clamp(volts, -Constants.Module.NOMINAL_VOLTAGE, Constants.Module.NOMINAL_VOLTAGE));
  }

  /**
   * The static-friction term: enough voltage to break the module loose, in whichever direction it
   * needs to go, and nothing at all once it is close enough.
   *
   * <p>This exists because proportional action cannot solve stiction here. A module parks where the
   * voltage its error produces drops below its breakaway, so shrinking that parked error by gain
   * alone needs a kP around 12 on the stiffest corner -- and the turn loop goes unstable somewhere
   * near there, because it closes over a 50 Hz link whose worst-case staleness is around 160 ms.
   * kP 8 was enough to make the modules spin continuously rather than settle. Adding a fixed push
   * instead defeats friction without touching loop gain, so it costs no stability margin.
   *
   * <p>The tolerance band is not a nicety. A fixed push that never switches off drives past the
   * setpoint, gets pushed back, and hunts forever; dropping the term once inside the band is what
   * makes the module settle, and it sets the accuracy the loop converges to.
   *
   * @param errorRad wrapped position error, positive when the module must turn positive
   * @param ks this corner's breakaway voltage
   * @param toleranceRad half-width of the band inside which no push is applied
   * @return volts to add to the proportional output
   */
  static double turnFeedforwardVolts(double errorRad, double ks, double toleranceRad) {
    if (Math.abs(errorRad) <= toleranceRad) {
      return 0.0;
    }
    return Math.copySign(ks, errorRad);
  }

  private Rotation2d readAbsolutePosition() {
    EncodersFrame encoders = rioBridgeCan == null ? null : rioBridgeCan.latestEncoders();
    if (encoders == null) {
      // No RioBridge, or it hasn't sent an Encoders frame yet (e.g. still booting): the relative
      // encoder, zeroed at alignment, is the only heading there is.
      return new Rotation2d(turnEncoder.getPosition().get(0.0));
    }
    double radians = adcCountToTurnFraction(encoders.rawCounts()[encoderChannel]) * 2 * Math.PI;
    return new Rotation2d(MathUtil.angleModulus(radians)).minus(absoluteEncoderOffset);
  }

  /**
   * Converts a RioBridge raw 12-bit ADC count (0-4095) into the same 0-1 fraction-of-a-turn
   * {@code AnalogEncoder.get()} would have returned with the single-argument constructor this
   * replaced (fullRange 1, expectedZero 0), so the offset math above didn't need to change.
   * Package-visible for {@code RioBridgeEncoderConversionTest}.
   */
  static double adcCountToTurnFraction(int rawCount) {
    return rawCount / ENCODER_ADC_COUNTS;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    Signal<Double> drivePosition = driveEncoder.getPosition();
    Signal<Double> driveVelocity = driveEncoder.getVelocity();
    Signal<Double> driveOutput = driveSpark.getAppliedOutput();
    Signal<Double> driveBusVolts = driveSpark.getBusVoltage();

    inputs.driveConnected = drivePosition.isValid() && driveVelocity.isValid();
    inputs.drivePositionRad = drivePosition.get(inputs.drivePositionRad);
    inputs.driveVelocityRadPerSec = driveVelocity.get(inputs.driveVelocityRadPerSec);
    inputs.driveAppliedVolts = driveOutput.get(0.0) * driveBusVolts.get(0.0);
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent().get(0.0);

    Signal<Double> turnPosition = turnEncoder.getPosition();
    Signal<Double> turnVelocity = turnEncoder.getVelocity();
    Signal<Double> turnOutput = turnSpark.getAppliedOutput();
    Signal<Double> turnBusVolts = turnSpark.getBusVoltage();

    inputs.turnConnected = turnPosition.isValid() && turnVelocity.isValid();
    // Unlike a directly-wired analog encoder, a RioBridge-sourced reading has a real staleness
    // signal: the Encoders frame's own CAN timestamp. False means either there's no RioBridge at
    // all, or its last Encoders frame is more than 100 ms old -- either way, the heading is only
    // as good as the last manual zeroing, which is worth seeing in the log.
    inputs.turnEncoderConnected =
        rioBridgeCan != null
            && (Timer.getMonotonicTimestamp() - rioBridgeCan.latestEncodersTimestampSeconds())
                < ENCODER_STALE_THRESHOLD_SECONDS;
    inputs.turnAbsolutePosition = readAbsolutePosition();
    // Control works from the module, not the motor -- see ModuleIOInputs.turnPosition.
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnMotorPosition = new Rotation2d(turnPosition.get(0.0));
    runTurnControl(
        inputs.turnAbsolutePosition, inputs.turnEncoderConnected, inputs.turnMotorPosition);
    inputs.turnVelocityRadPerSec = turnVelocity.get(inputs.turnVelocityRadPerSec);
    inputs.turnAppliedVolts = turnOutput.get(0.0) * turnBusVolts.get(0.0);
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent().get(0.0);
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    // Hand control back from the position loop, or the two write opposing voltages every cycle.
    turnSetpoint = null;
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double feedforwardVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    // Only records the target. The loop itself runs in updateInputs(), so that every cycle it acts
    // on the absolute reading taken that same cycle rather than one from wherever in the loop this
    // happened to be called.
    turnSetpoint = rotation;
  }

  @Override
  public void setDriveGains(double kP, double kD, double kS, double kV) {
    driveKs = kS;
    driveKv = kV;
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(voltsPerErrorToDuty(kP), 0.0, voltsPerErrorToDuty(kD));
    // kNoPersistParameters so a tuning session does not burn every edit to flash; once the numbers
    // are settled they belong in Constants, not in the controller's memory.
    driveSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setTurnGains(double kP, double kD, double kS, double toleranceRad) {
    // Retunes the local controller rather than reconfiguring the SPARK: the turn loop no longer
    // runs on the controller, so there is nothing to push over CAN and a dashboard edit takes
    // effect on the very next cycle. Gains stay in volts per radian, unconverted.
    turnController.setPID(kP, 0.0, kD);
    turnKs = kS;
    turnFeedforwardToleranceRad = toleranceRad;
  }

  @Override
  public void zeroTurnEncoder() {
    turnEncoder.setPosition(0.0);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    // Only touching idle mode, so leave the rest of the configuration where it is.
    driveSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
