// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * One swerve module's logic, sitting on top of a {@link ModuleIO}.
 *
 * <p>This class holds everything that is the same whether the module is real or simulated: unit
 * conversion between wheel radians and metres, angle optimization, and publishing telemetry. It
 * never touches a motor controller directly.
 */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }

  /**
   * Reads hardware and publishes this module's inputs. Called once per loop from {@link Drive}
   * before any control runs, so every calculation in a given loop sees the same snapshot.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
    // Logger.processInputs writes these values to the log when running, and replaces them with the
    // logged values when replaying. That single call is what makes replay work.
    Logger.processInputs("Drive/" + name, inputs);

    Logger.recordOutput("Drive/" + name + "/DrivePositionMeters", getPositionMeters());
    Logger.recordOutput("Drive/" + name + "/DriveVelocityMetersPerSec", getVelocityMetersPerSec());
    Logger.recordOutput("Drive/" + name + "/TurnPositionDeg", getAngle().getDegrees());
  }

  /**
   * Commands a module state, taking the shorter path to the requested angle.
   *
   * @param state the desired wheel speed and module heading
   */
  public void runSetpoint(SwerveModuleVelocity state) {
    SwerveModuleVelocity optimized = state.optimize(getAngle());
    // Scale the wheel speed down while the module is still turning into place, so the robot does not
    // lurch sideways during the first few milliseconds of a direction change.
    optimized = optimized.cosineScale(getAngle());

    io.setDriveVelocity(optimized.velocity / Constants.Module.WHEEL_RADIUS);
    io.setTurnPosition(optimized.angle);
  }

  /**
   * Runs the drive motor open-loop while holding the module straight ahead. Used by the
   * characterization routine.
   *
   * @param volts voltage to apply to the drive motor
   */
  public void runCharacterization(double volts) {
    io.setDriveOpenLoop(volts);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Stops both motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Switches the drive motor between brake and coast. */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Current module heading. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Distance the wheel has travelled, in metres. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * Constants.Module.WHEEL_RADIUS;
  }

  /** Current wheel speed, in metres per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * Constants.Module.WHEEL_RADIUS;
  }

  /** Wheel travel and heading, for odometry. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Wheel speed and heading, for chassis-velocity estimation. */
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Wheel travel in radians, used by the characterization routine. */
  public double getWheelRadiansForCharacterization() {
    return inputs.drivePositionRad;
  }

  /** True when both controllers answered on the last cycle. */
  public boolean isConnected() {
    return inputs.driveConnected && inputs.turnConnected;
  }
}
