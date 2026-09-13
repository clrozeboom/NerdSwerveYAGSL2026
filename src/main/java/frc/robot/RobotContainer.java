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
import frc.robot.subsystems.drive.riobridge.GyroIORioBridge;
import frc.robot.subsystems.drive.riobridge.RioBridgeCan;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.framework.RobotBase;

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
  // alpha-7 deleted SendableChooser. AdvantageKit's replacement takes the NetworkTables key in
  // its constructor, so there is no separate publish step, and the selection is written to the log
  // and fed back on replay — which a bare tunable Selectable would not do.
  private final LoggedNetworkChooser<Command> autoChooser =
      new LoggedNetworkChooser<>("Auto Chooser");

  public RobotContainer() {
    if (RobotBase.isReal()) {
      // The NavX the YAGSL config used has no 2027 release, and SystemCore dropped SPI
      // regardless -- same story for the four Thrifty absolute encoders, which never had
      // anywhere on this SystemCore to plug into. The RioBridge (see Constants.RioBridge) is a
      // roboRIO that runs both under their unmodified 2026 vendor libraries and republishes the
      // readings over CAN; one shared instance feeds both the gyro and every module's absolute
      // encoder below. If this robot ever drops the RioBridge, revert to GyroIONone/GyroIOOnboard
      // and pass null here, and flip Constants.Module.HAS_ABSOLUTE_ENCODERS back to false.
      RioBridgeCan rioBridgeCan = new RioBridgeCan(Constants.RioBridge.BUS_ID);
      drive =
          new Drive(
              new GyroIORioBridge(rioBridgeCan),
              new ModuleIOSpark(ModuleConfig.FRONT_LEFT, rioBridgeCan),
              new ModuleIOSpark(ModuleConfig.FRONT_RIGHT, rioBridgeCan),
              new ModuleIOSpark(ModuleConfig.BACK_LEFT, rioBridgeCan),
              new ModuleIOSpark(ModuleConfig.BACK_RIGHT, rioBridgeCan));
    } else {
      // The RioBridge is real hardware this project has no simulation model for, so simulation
      // keeps the wheel-integrated fallback (GyroIONone) rather than pretending to have a gyro.
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
    // Still the default even with the RioBridge gyro wired up above: this binding predates it and
    // switching the default drive scheme is a real behavior change worth deciding on its own,
    // rather than as a side effect of adding gyro support. Left bumper below now gets a
    // non-drifting heading either way -- consider promoting it to the default once that's been
    // driven and confirmed to feel right.
    drive.setDefaultCommand(
        DriveCommands.robotRelativeDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Hold the left bumper for field-relative.
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

    // Hold the modules in an X to resist being pushed.
    driver.faceRight().whileTrue(DriveCommands.stopWithX(drive));

    // Call the direction the robot currently faces "forward".
    driver.start().onTrue(Commands.runOnce(drive::zeroHeading).ignoringDisable(true));

    // Back re-zeroes the modules, for the common case of having straightened the wheels by hand
    // after pushing the robot around. Only acts while disabled — see TuningCommands.zeroModules.
    // Bound unconditionally rather than behind TUNING_MODE because without absolute encoders this
    // is a normal part of operating the robot, not a tuning aid.
    driver.back().onTrue(TuningCommands.zeroModules(drive));
  }

  private void configureAutoChooser() {
    autoChooser.addDefault("Do Nothing", Commands.none());
    // Bring-up routines. These live on the auto chooser because that is the one place a command can
    // be picked and run without a controller binding; see TuningCommands for what each one measures
    // and which of them move the robot.
    if (Constants.TUNING_MODE) {
      // Listed in the order the README's bring-up sequence works through them: everything that
      // fits in a metre of clearance first, then the two that need a runway.
      if (Constants.Module.HAS_ABSOLUTE_ENCODERS) {
        autoChooser.add("Tuning 1: Report Encoder Offsets", TuningCommands.reportEncoderOffsets(drive));
      } else {
        autoChooser.add("Tuning 1: Zero Modules (align wheels first)", TuningCommands.zeroModules(drive));
      }
      autoChooser.add("Tuning 2: Feedforward Ramp (quick)", TuningCommands.feedforwardRamp(drive));
      autoChooser.add("Tuning 2: Steady-State Sweep (kS/kV)", TuningCommands.steadyStateSweep(drive));
      autoChooser.add("Tuning 2: Spin SysId (all four)", TuningCommands.spinSysIdFull(drive));
      autoChooser.add("Tuning 3: Spin Step Response", TuningCommands.spinStepResponse(drive));
      autoChooser.add("Tuning 3: Turn Step Response", TuningCommands.turnStepResponse(drive));
      autoChooser.add("Tuning 4: Measure Wheel Radius", TuningCommands.measureWheelRadius(drive));
      autoChooser.add("Tuning 5: Drive Square (odometry check)", TuningCommands.driveSquare(drive));
      autoChooser.add("Tuning (opt): Drive Step Response", TuningCommands.driveStepResponse(drive));
      autoChooser.add("Tuning (opt): Drive SysId (all four)", TuningCommands.driveSysIdFull(drive));
    }
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
