// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
 * <p><b>Unverified against real hardware.</b> REVLib 2027.0.0-alpha-6 is published for SystemCore
 * but its jar could not be inspected while this was written, and the REVLib API has changed shape
 * between seasons before. The calls below follow the 2025/2026 REVLib API. If any of them do not
 * resolve, this file is the only place that needs fixing — every other class talks to
 * {@link ModuleIO}, not to REVLib.
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

  public ModuleIOSpark(ModuleConfig config) {
    driveSpark = new SparkMax(config.driveCanId, MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnCanId, MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    absoluteEncoder = new AnalogEncoder(config.encoderChannel);
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

    // Seed the turn motor's relative encoder from the absolute encoder so the module knows where it
    // is pointing at power-on.
    turnEncoder.setPosition(readAbsolutePosition().getRadians());
    driveEncoder.setPosition(0.0);
  }

  private Rotation2d readAbsolutePosition() {
    // AnalogEncoder.get() returns a 0-1 fraction of a full turn with the single-argument
    // constructor used above (fullRange 1, expectedZero 0).
    double radians = absoluteEncoder.get() * 2 * Math.PI;
    return new Rotation2d(MathUtil.angleModulus(radians)).minus(absoluteEncoderOffset);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveSpark.getLastError() == REVLibError.kOk;
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();

    inputs.turnConnected = turnSpark.getLastError() == REVLibError.kOk;
    // An analog encoder is just a voltage, so there is no connection status to read. A
    // disconnected one reads a constant value rather than reporting an error, which shows up as
    // a module whose absolute position never changes.
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = readAbsolutePosition();
    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();
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
    double feedforwardVolts =
        Constants.Module.DRIVE_KS * Math.signum(velocityRadPerSec)
            + Constants.Module.DRIVE_KV * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnController.setReference(
        MathUtil.angleModulus(rotation.getRadians()), ControlType.kPosition);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    // Only touching idle mode, so leave the rest of the configuration where it is.
    driveSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
