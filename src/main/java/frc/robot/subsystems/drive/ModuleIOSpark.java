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
import org.wpilib.hardware.rotation.AnalogEncoder;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Real-hardware module: two SPARK MAX / NEO pairs plus a Thrifty analog absolute encoder, matching
 * the hardware described by the YAGSL {@code deploy/swerve/neo} config.
 *
 * <p>Closed-loop control runs on the SPARK MAX itself, which is why this class configures gains onto
 * the controller rather than owning PID objects the way {@link ModuleIOSim} does.
 *
 * <p>REVLib 2027 reshaped its read API: every getter now returns a {@link Signal}, which carries the
 * value together with a timestamp and a validity flag rather than a bare double. That is what feeds
 * the {@code *Connected} inputs below — a stale or errored signal is exactly the "controller did not
 * answer this cycle" case the IO layer wants to report. Commanding a closed-loop setpoint is
 * {@code setSetpoint} here, not the {@code setReference} of previous seasons.
 */
public class ModuleIOSpark implements ModuleIO {
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
  private final AnalogEncoder absoluteEncoder;
  private final Rotation2d absoluteEncoderOffset;

  private double driveKs = Constants.Module.DRIVE_KS;
  private double driveKv = Constants.Module.DRIVE_KV;

  public ModuleIOSpark(ModuleConfig config) {
    driveSpark = new SparkMax(Constants.Module.CAN_BUS_ID, config.driveCanId, MotorType.kBrushless);
    turnSpark = new SparkMax(Constants.Module.CAN_BUS_ID, config.turnCanId, MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Only touch the analog input when the encoder is actually wired. On a SystemCore with nothing
    // on that channel there is no reading to take, and constructing the input would claim hardware
    // for no reason.
    absoluteEncoder =
        Constants.Module.HAS_ABSOLUTE_ENCODERS ? new AnalogEncoder(config.encoderChannel) : null;
    absoluteEncoderOffset = Rotation2d.fromDegrees(config.absoluteEncoderOffsetDegrees);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Module.DRIVE_CURRENT_LIMIT)
        .voltageCompensation(Constants.Module.NOMINAL_VOLTAGE)
        .openLoopRampRate(Constants.Module.DRIVE_RAMP_RATE);
    driveConfig.encoder.positionConversionFactor(DRIVE_POSITION_FACTOR);
    driveConfig.encoder.velocityConversionFactor(DRIVE_VELOCITY_FACTOR);
    driveConfig.closedLoop.pid(Constants.Module.DRIVE_KP, 0.0, Constants.Module.DRIVE_KD);
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
    turnConfig.closedLoop.pid(Constants.Module.TURN_KP, 0.0, Constants.Module.TURN_KD);
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // With an absolute encoder the module can work out where it is pointing on its own. Without one
    // it has to be told, so assume it starts aligned — whoever powered the robot on is expected to
    // have straightened the wheels, and the "Zero Modules" routine re-asserts it on demand.
    turnEncoder.setPosition(
        Constants.Module.HAS_ABSOLUTE_ENCODERS ? readAbsolutePosition().getRadians() : 0.0);
    driveEncoder.setPosition(0.0);
  }

  private Rotation2d readAbsolutePosition() {
    if (absoluteEncoder == null) {
      // No absolute encoder: the relative encoder, zeroed at alignment, is the only heading there is.
      return new Rotation2d(turnEncoder.getPosition().get(0.0));
    }
    // AnalogEncoder.get() returns a 0-1 fraction of a full turn with the single-argument
    // constructor used above (fullRange 1, expectedZero 0).
    double radians = absoluteEncoder.get() * 2 * Math.PI;
    return new Rotation2d(MathUtil.angleModulus(radians)).minus(absoluteEncoderOffset);
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
    // An analog encoder is just a voltage, so there is no connection status to read. A
    // disconnected one reads a constant value rather than reporting an error, which shows up as
    // a module whose absolute position never changes. False here means there is no encoder at all,
    // which is worth seeing in the log because it says the heading is only as good as the last
    // manual zeroing.
    inputs.turnEncoderConnected = Constants.Module.HAS_ABSOLUTE_ENCODERS;
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
    config.closedLoop.pid(kP, 0.0, kD);
    // kNoPersistParameters so a tuning session does not burn every edit to flash; once the numbers
    // are settled they belong in Constants, not in the controller's memory.
    driveSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setTurnGains(double kP, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, 0.0, kD);
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
