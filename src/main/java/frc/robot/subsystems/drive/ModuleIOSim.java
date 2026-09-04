// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.util.Nat;
import org.wpilib.simulation.DCMotorSim;

/**
 * Physics-sim module, used when the code runs on a desktop rather than a SystemCore.
 *
 * <p>Both motors are modelled as DC motors with the real gear ratios from the YAGSL config, so the
 * numbers coming out of here are in the same units and roughly the same range as the real robot's.
 * Closed-loop control runs here rather than on a motor controller, which is why this class owns its
 * own PID controllers while {@link ModuleIOSpark} hands that job to the SPARK MAX.
 */
public class ModuleIOSim implements ModuleIO {
  private static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);
  private static final DCMotor TURN_MOTOR = DCMotor.getNEO(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private final PIDController driveController =
      new PIDController(Constants.Module.DRIVE_KP, 0.0, Constants.Module.DRIVE_KD);
  private final PIDController turnController =
      new PIDController(Constants.Module.TURN_KP, 0.0, Constants.Module.TURN_KD);

  /** Width of the velocity band over which static friction is blended in; see {@link #driveFrictionVolts()}. */
  private static final double FRICTION_BLEND_RAD_PER_SEC = 0.25;

  private double driveKs = Constants.Module.DRIVE_KS;
  private double driveKv = Constants.Module.DRIVE_KV;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFeedforwardVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    driveSim =
        new DCMotorSim(
            dcMotorSystem(
                DRIVE_MOTOR, Constants.Module.DRIVE_SIM_MOI, Constants.Module.DRIVE_GEAR_RATIO),
            DRIVE_MOTOR);
    turnSim =
        new DCMotorSim(
            dcMotorSystem(
                TURN_MOTOR, Constants.Module.TURN_SIM_MOI, Constants.Module.TURN_GEAR_RATIO),
            TURN_MOTOR);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Builds the state-space model of a geared DC motor: state is [position, velocity], input is
   * volts, output is [position, velocity].
   *
   * <p>WPILib 2026 had {@code LinearSystemId.createDCMotorSystem} for this; 2027 removed
   * {@code LinearSystemId}, so the two matrices are written out here instead.
   */
  private static LinearSystem<N2, N1, N2> dcMotorSystem(
      DCMotor motor, double momentOfInertia, double gearing) {
    DCMotor geared = motor.withReduction(gearing);
    return new LinearSystem<>(
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0.0,
            1.0,
            0.0,
            -geared.Kt / (geared.Kv * geared.R * momentOfInertia)),
        MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, geared.Kt / (geared.R * momentOfInertia)),
        MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
        MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFeedforwardVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // DCMotorSim is frictionless, but kS exists precisely to overcome static friction. Feeding a
    // real robot's kS into a frictionless plant is surplus voltage with nothing to cancel it, so
    // model the friction it compensates for: a constant torque opposing motion, expressed as the
    // voltage it takes to overcome. Without this the wheel settles at over twice the commanded
    // speed whenever the feedback gain is small.
    driveSim.setInputVoltage(clampToBattery(driveAppliedVolts - driveFrictionVolts()));
    turnSim.setInputVoltage(clampToBattery(turnAppliedVolts));
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());
  }

  /**
   * The voltage-equivalent of the drive wheel's static friction, opposing whichever way it turns.
   *
   * <p>A plain {@code signum} would be the obvious way to write this and is wrong: it flips sign
   * every timestep once the wheel is nearly stopped, so the wheel chatters around zero and never
   * actually settles. Blending across a small velocity band removes the discontinuity, at the cost
   * of letting the last fraction of a rad/s bleed off smoothly rather than stopping dead.
   */
  private double driveFrictionVolts() {
    double omega = driveSim.getAngularVelocity();
    return driveKs * Math.tanh(omega / FRICTION_BLEND_RAD_PER_SEC);
  }

  private static double clampToBattery(double volts) {
    // WPILib 2027 dropped MathUtil.clamp in favour of the JDK's own.
    return Math.clamp(volts, -Constants.Module.NOMINAL_VOLTAGE, Constants.Module.NOMINAL_VOLTAGE);
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = volts;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFeedforwardVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void zeroTurnEncoder() {
    turnSim.setAngle(0.0);
    turnController.setSetpoint(0.0);
  }

  @Override
  public void setDriveGains(double kP, double kD, double kS, double kV) {
    driveController.setPID(kP, 0.0, kD);
    driveKs = kS;
    driveKv = kV;
  }

  @Override
  public void setTurnGains(double kP, double kD) {
    turnController.setPID(kP, 0.0, kD);
  }
}
