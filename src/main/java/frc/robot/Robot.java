// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;

/**
 * Robot lifecycle.
 *
 * <p>Extends AdvantageKit's {@link LoggedRobot} rather than {@code TimedRobot}. The difference that
 * matters is timing: {@code LoggedRobot} drives the loop from the log during replay instead of from
 * the system clock, which is what lets a replay run produce the same results as the original.
 *
 * <p>Two 2027 changes from the usual shape: {@code robotInit()} no longer exists, so setup happens
 * in the constructor, and "Test" mode is now called "Utility", so the hooks are
 * {@code utilityInit}/{@code utilityPeriodic}.
 */
public class Robot extends LoggedRobot {
  /**
   * Set to a log file path to replay it instead of running normally. Leave null for real operation
   * and ordinary simulation.
   *
   * <p>Replay only makes sense on a desktop — point this at a {@code .wpilog} pulled off the robot,
   * run the sim, and every {@code updateInputs} call is fed from the log rather than from hardware.
   * The control code then re-executes against exactly the inputs it saw on the field, and the
   * outputs land in a new log beside the original for comparison in AdvantageScope.
   */
  private static final String REPLAY_LOG = null;

  /** Set true to write a log during ordinary simulation, so a sim run can itself be replayed. */
  private static final boolean LOG_IN_SIM = false;

  private final RobotContainer robotContainer;
  private final Timer disabledTimer = new Timer();

  private Command autonomousCommand;

  public Robot() {
    configureLogging();
    robotContainer = new RobotContainer();
  }

  /**
   * Wires up AdvantageKit before anything else runs. Exactly one of two modes is chosen here: either
   * the robot is running and inputs come from hardware while being written to a log, or it is
   * replaying and inputs come from a log instead.
   */
  private void configureLogging() {
    Logger.recordMetadata("ProjectName", "NerdSwerve2027");
    Logger.recordMetadata("RuntimeType", RobotBase.getRuntimeType().toString());

    if (REPLAY_LOG == null) {
      if (RobotBase.isReal()) {
        // Log to the USB stick if one is mounted, and publish live to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      } else {
        // Sim normally just publishes live for AdvantageScope. Flip LOG_IN_SIM to capture a
        // .wpilog you can then feed back through REPLAY_LOG above.
        if (LOG_IN_SIM) {
          Logger.addDataReceiver(new WPILOGWriter("logs"));
        }
        Logger.addDataReceiver(new NT4Publisher());
      }
    } else {
      // Replay: feed inputs from the log and write the recomputed outputs to a sibling file.
      setUseTiming(false);
      Logger.setReplaySource(new WPILOGReader(REPLAY_LOG));
      Logger.addDataReceiver(new WPILOGWriter(REPLAY_LOG.replaceFirst("\\.wpilog$", "") + "_replay.wpilog"));
    }

    Logger.start();
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
