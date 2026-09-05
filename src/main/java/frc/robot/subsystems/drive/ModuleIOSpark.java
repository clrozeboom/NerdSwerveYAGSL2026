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
  private final SparkClosedLoopController turnController;
  private final RioBridgeCan rioBridgeCan;
  private final int encoderChannel;
  private final Rotation2d absoluteEncoderOffset;

  private double driveKs = Constants.Module.DRIVE_KS;
  private double driveKv = Constants.Module.DRIVE_KV;

  /**
   * Converts a gain expressed in volts per unit of error into the duty cycle a SPARK MAX closed
   * loop wants.
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
    turnController = turnSpark.getClosedLoopController();

    this.rioBridgeCan = Constants.Module.HAS_ABSOLUTE_ENCODERS ? rioBridgeCan : null;
    this.encoderChannel = config.encoderChannel;
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
    // The module wraps, so let the controller take the short way round rather than unwinding.
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
    turnConfig.closedLoop.pid(
        voltsPerErrorToDuty(Constants.Module.TURN_KP),
        0.0,
        voltsPerErrorToDuty(Constants.Module.TURN_KD));
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
    inputs.turnPosition = new Rotation2d(turnPosition.get(0.0));
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
    turnController.setSetpoint(
        MathUtil.angleModulus(rotation.getRadians()), ControlType.kPosition);
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
  public void setTurnGains(double kP, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(voltsPerErrorToDuty(kP), 0.0, voltsPerErrorToDuty(kD));
    turnSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
