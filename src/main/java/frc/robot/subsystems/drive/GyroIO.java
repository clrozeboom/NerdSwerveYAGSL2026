// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Hardware abstraction for the drivetrain gyro, in the same shape as {@link ModuleIO}.
 *
 * <p>The YAGSL project this was derived from used a NavX, which has no 2027 release and which
 * SystemCore could not talk to anyway now that SPI has been removed. Two implementations ship here:
 * {@link GyroIONone}, which reports permanently disconnected, and {@link GyroIOOnboard}, which uses
 * SystemCore's built-in IMU. {@link Drive} handles {@code connected == false} by integrating module
 * headings instead, so the drivetrain is fully usable either way.
 */
public interface GyroIO {

  /** Everything read from the gyro in a single loop. */
  @AutoLog
  class GyroIOInputs {
    /** True when the gyro is present and returning usable data. */
    public boolean connected = false;

    /** Robot heading, CCW-positive. */
    public Rotation2d yawPosition = Rotation2d.kZero;

    /** Yaw rate, in radians per second, CCW-positive. */
    public double yawVelocityRadPerSec = 0.0;
  }

  /** Refreshes {@code inputs} from hardware. Called exactly once per loop, before anything else. */
  default void updateInputs(GyroIOInputs inputs) {}

  /** Zeroes the reported yaw. No-op when there is no gyro. */
  default void resetYaw() {}
}
