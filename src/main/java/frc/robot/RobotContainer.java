// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ModuleConfig;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONone;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.framework.RobotBase;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

/**
 * Wires the robot together: picks the IO implementations for the current environment, builds the
 * subsystems, and binds the controls.
 *
 * <p>This is the only place that knows whether the code is talking to real hardware or to the
 * simulator. Everything below {@link Drive} takes its IO as a constructor argument.
 */
public class RobotContainer {
  private final CommandGamepad driver = new CommandGamepad(0);
  private final Drive drive;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    if (RobotBase.isReal()) {
      drive =
          new Drive(
              // No gyro yet. The NavX the YAGSL config used has no 2027 release, and SystemCore
              // dropped SPI regardless. Swap in GyroIOOnboard to use SystemCore's built-in IMU
              // once its mount orientation is confirmed; Drive works either way.
              new GyroIONone(),
              new ModuleIOSpark(ModuleConfig.FRONT_LEFT),
              new ModuleIOSpark(ModuleConfig.FRONT_RIGHT),
              new ModuleIOSpark(ModuleConfig.BACK_LEFT),
              new ModuleIOSpark(ModuleConfig.BACK_RIGHT));
    } else {
      drive =
          new Drive(
              new GyroIONone(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
    }

    configureAutoChooser();
    configureBindings();
  }

  private void configureBindings() {
    // Robot-relative driving on the sticks. Left stick translates, right stick turns. Joystick axes
    // are negated because pushing a stick forward or left reads negative.
    //
    // This is the default rather than field-relative because there is no gyro on this robot. With
    // heading coming only from integrated module positions it drifts, and a drifting heading makes
    // field-relative driving progressively wrong in a way that is confusing to drive. Robot-relative
    // ignores heading entirely, so it stays correct indefinitely.
    drive.setDefaultCommand(
        DriveCommands.robotRelativeDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Hold the left bumper for field-relative. Only useful once a gyro is fitted; until then the
    // heading it works from is the drifting wheel-derived estimate.
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

    // Hold the modules in an X to resist being pushed.
    driver.eastFace().whileTrue(DriveCommands.stopWithX(drive));

    // Call the direction the robot currently faces "forward".
    driver.start().onTrue(Commands.runOnce(drive::zeroHeading).ignoringDisable(true));
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Drive Feedforward Characterization",
        DriveCommands.feedforwardCharacterization(drive, 0.5, 6.0));

    // Bring-up routines. These live on the auto chooser because that is the one place a command can
    // be picked and run without a controller binding; see TuningCommands for what each one measures
    // and which of them move the robot.
    if (Constants.TUNING_MODE) {
      autoChooser.addOption("Tuning: Report Encoder Offsets", TuningCommands.reportEncoderOffsets(drive));
      autoChooser.addOption("Tuning: Turn Step Response", TuningCommands.turnStepResponse(drive));
      autoChooser.addOption("Tuning: Drive Step Response", TuningCommands.driveStepResponse(drive));
      autoChooser.addOption("Tuning: Measure Wheel Radius", TuningCommands.measureWheelRadius(drive));
      autoChooser.addOption("Tuning: Drive SysId (all four)", TuningCommands.driveSysIdFull(drive));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /** The command to run in autonomous, from the dashboard chooser. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Puts the drive motors into brake or coast. */
  public void setMotorBrake(boolean brake) {
    drive.setBrakeMode(brake);
  }
}
