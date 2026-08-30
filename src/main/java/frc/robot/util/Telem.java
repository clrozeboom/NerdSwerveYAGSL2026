// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.wpilib.smartdashboard.SmartDashboard;

/**
 * One place for the project's telemetry writes.
 *
 * <p>Two reasons this exists rather than calling {@code SmartDashboard} everywhere. First,
 * {@code SmartDashboard} and the whole {@code Sendable} interface are deleted in WPILib
 * 2027 alpha-7 in favour of {@code org.wpilib.telemetry.Telemetry}; keeping the calls behind this
 * facade turns that migration into a one-file change. Second, it is the seam where AdvantageKit's
 * {@code Logger} would go if this project later takes on that dependency.
 */
public final class Telem {
  private Telem() {}

  /** Publishes a numeric value. */
  public static void log(String key, double value) {
    SmartDashboard.putNumber(key, value);
  }

  /** Publishes a boolean value. */
  public static void log(String key, boolean value) {
    SmartDashboard.putBoolean(key, value);
  }

  /** Publishes a string value. */
  public static void log(String key, String value) {
    SmartDashboard.putString(key, value);
  }

  /** Publishes an array of numbers. */
  public static void log(String key, double[] values) {
    SmartDashboard.putNumberArray(key, values);
  }
}
