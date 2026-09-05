package frc.robot.subsystems.drive.riobridge.diagnostics;

import org.wpilib.framework.RobotBase;

/**
 * Entry point for {@link DiagnosticsRobot}. See RioBridge's docs/hardware-verification.md: point
 * this project's actual {@code frc.robot.Main} at {@code DiagnosticsRobot.class} for one bench
 * session instead of adding a second {@code main()} -- this class exists so the diagnostics
 * package has one ready to copy verbatim if that's easier for your project layout. (This
 * project's alpha-6 WPILib pin takes the robot class itself here, not a supplier -- see the real
 * {@code Main.java}'s own comment on that.)
 */
public final class DiagnosticsMain {
  private DiagnosticsMain() {}

  public static void main(String[] args) {
    RobotBase.startRobot(DiagnosticsRobot.class);
  }
}
