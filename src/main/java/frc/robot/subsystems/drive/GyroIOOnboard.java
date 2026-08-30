// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

/**
 * Gyro implementation backed by SystemCore's built-in IMU.
 *
 * <p>This is the drop-in replacement for the NavX the YAGSL project used. Swap
 * {@link GyroIONone} for this in {@code RobotContainer} once the mounting orientation below has
 * been confirmed against how the SystemCore actually sits on the robot — {@code FLAT} means the
 * board is mounted horizontally.
 */
public class GyroIOOnboard implements GyroIO {
  private final OnboardIMU imu;

  public GyroIOOnboard() {
    this(MountOrientation.FLAT);
  }

  public GyroIOOnboard(MountOrientation orientation) {
    imu = new OnboardIMU(orientation);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = imu.getRotation2d();
    // getGyroRateZ() is the raw IMU Z axis in rad/s, and unlike getRotation2d() it is not corrected
    // for mount orientation. That makes it the robot's yaw rate only when mounted FLAT; if you
    // switch to LANDSCAPE or PORTRAIT, read the matching axis here instead.
    inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
  }

  @Override
  public void resetYaw() {
    imu.resetYaw();
  }
}
