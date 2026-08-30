// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Timer;

/**
 * Robot lifecycle.
 *
 * <p>Two 2027 changes from the usual shape: {@code robotInit()} no longer exists, so setup happens
 * in the constructor, and "Test" mode is now called "Utility", so the hooks are
 * {@code utilityInit}/{@code utilityPeriodic}.
 */
public class Robot extends TimedRobot {
  private final RobotContainer robotContainer;
  private final Timer disabledTimer = new Timer();

  private Command autonomousCommand;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, which polls triggers, runs scheduled commands, and calls every
    // subsystem's periodic(). Nothing in the command framework works without this.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Hold brake briefly so the robot stops where it is, then coast so it can be pushed around.
    robotContainer.setMotorBrake(true);
    disabledTimer.restart();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)) {
      robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.setMotorBrake(true);
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    robotContainer.setMotorBrake(true);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
