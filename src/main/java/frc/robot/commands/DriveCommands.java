// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/** Command factories for the drivetrain. */
public final class DriveCommands {
  private DriveCommands() {}

  /**
   * Field-relative joystick drive.
   *
   * <p>The translation deadband is applied to the joystick's distance from centre rather than to
   * each axis on its own, so a diagonal push near the deadband edge does not get squared off into a
   * cardinal direction. Magnitude is then squared to give finer control at low speed.
   *
   * @param drive the drivetrain
   * @param xSupplier forward axis, +1 is away from the driver station
   * @param ySupplier left axis, +1 is to the driver's left
   * @param omegaSupplier rotation axis, +1 is counter-clockwise
   * @return a command that drives until interrupted
   */
  public static Command joystickDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linear =
              applyRadialDeadband(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Operator.DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          ChassisVelocities fieldRelative =
              new ChassisVelocities(
                  linear.getX() * drive.getMaxLinearSpeed(),
                  linear.getY() * drive.getMaxLinearSpeed(),
                  omega * drive.getMaxAngularSpeed());

          drive.runVelocity(fieldRelative.toRobotRelative(drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot-relative joystick drive, for when the heading estimate is not trusted — which includes any
   * time the robot is running without a gyro.
   *
   * @param drive the drivetrain
   * @param xSupplier forward axis, +1 is robot forward
   * @param ySupplier left axis, +1 is robot left
   * @param omegaSupplier rotation axis, +1 is counter-clockwise
   * @return a command that drives until interrupted
   */
  public static Command robotRelativeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linear =
              applyRadialDeadband(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Operator.DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          drive.runVelocity(
              new ChassisVelocities(
                  linear.getX() * drive.getMaxLinearSpeed(),
                  linear.getY() * drive.getMaxLinearSpeed(),
                  omega * drive.getMaxAngularSpeed()));
        },
        drive);
  }

  /** Holds the modules in an X so the robot resists being pushed. */
  public static Command stopWithX(Drive drive) {
    return Commands.run(drive::stopWithX, drive);
  }

  /**
   * Applies the deadband to the joystick's distance from centre, then squares the magnitude while
   * keeping the direction.
   */
  private static Translation2d applyRadialDeadband(double x, double y) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.Operator.DEADBAND);
    if (magnitude == 0.0) {
      return Translation2d.kZero;
    }
    Rotation2d direction = new Rotation2d(x, y);
    double scaled = magnitude * magnitude;
    return new Translation2d(scaled * direction.getCos(), scaled * direction.getSin());
  }
}
