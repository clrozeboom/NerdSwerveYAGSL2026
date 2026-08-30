// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/**
 * Gyro implementation for a robot with no gyro wired up.
 *
 * <p>Leaves {@code connected} false, which tells {@link Drive} to track heading by integrating the
 * module positions through the kinematics instead. Field-relative driving still works; it just
 * drifts over time the way any wheel-only heading estimate does.
 */
public class GyroIONone implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
  }
}
