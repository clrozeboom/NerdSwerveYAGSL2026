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
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.RobotMode;

/**
 * Wires the robot together: picks the IO implementations for the current environment, builds the
 * subsystems, and binds the controls.
 *
 * <p>This is the only place that knows whether the code is talking to real hardware or to the
 * simulator. Everything below {@link Drive} takes its IO as a constructor argument.
 */
public class RobotContainer {
  /** Driver-station group for the routines that would be run in a match. */
  private static final String MATCH_GROUP = "Match";

  /** Driver-station group for the bring-up routines; see {@link TuningCommands}. */
  private static final String TUNING_GROUP = "Tuning";

  /**
   * One selectable autonomous routine.
   *
   * @param name what the driver station lists it as, and what identifies it when selected
   * @param group which heading the driver station files it under
   * @param command what to run
   */
  record AutoRoutine(String name, String group, Command command) {}

  private final CommandGamepad driver = new CommandGamepad(0);
  private final Drive drive;

  /** Every routine offered for autonomous, in the order the driver station lists them. */
  private final List<AutoRoutine> autoRoutines = new ArrayList<>();

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

    // Commands v3's Mechanism interface has no periodic() hook, so Drive's has to be handed to the
    // scheduler explicitly. Sideloads run before triggers are polled and before any command body,
    // which is the slot CommandScheduler used to call subsystem periodic() in -- so every command
    // still sees inputs read this same loop, as it did under v2.
    Scheduler.getDefault().addPeriodic(drive::periodic);
    Scheduler.getDefault()
        .addPeriodic(new DriverDisplay(drive, this::selectedRoutineName)::update);

    configureAutoRoutines();
    publishOpModes();
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

    // Call the direction the robot currently faces "forward". No ignoringDisable in v3: the
    // framework has no enabled/disabled gate at all, so this runs whenever the button is pressed.
    driver
        .start()
        .onTrue(Command.noRequirements(coroutine -> drive.zeroHeading()).named("Zero Heading"));

    // Back re-zeroes the modules, for the common case of having straightened the wheels by hand
    // after pushing the robot around. Only acts while disabled — see TuningCommands.zeroModules.
    // Bound unconditionally rather than behind TUNING_MODE because without absolute encoders this
    // is a normal part of operating the robot, not a tuning aid.
    driver.back().onTrue(TuningCommands.zeroModules(drive));
  }

  private void configureAutoRoutines() {
    // The first entry is the fallback: it is what runs if the driver station reports an operating
    // mode this code does not recognise, which RobotState.getOpMode() explicitly warns can happen.
    autoRoutines.add(
        new AutoRoutine(
            "Do Nothing", MATCH_GROUP, Command.noRequirements(coroutine -> {}).named("Do Nothing")));

    // Bring-up routines. Under the chooser these needed AdvantageScope or Elastic open to reach;
    // as operating modes they are pickable from the driver station itself, which is where whoever
    // is running them is already standing. See TuningCommands for what each one measures and which
    // of them move the robot.
    if (Constants.TUNING_MODE) {
      // Listed in the order the README's bring-up sequence works through them: everything that
      // fits in a metre of clearance first, then the two that need a runway.
      if (Constants.Module.HAS_ABSOLUTE_ENCODERS) {
        addTuning("1: Report Encoder Offsets", TuningCommands.reportEncoderOffsets(drive));
      } else {
        addTuning("1: Zero Modules (align wheels first)", TuningCommands.zeroModules(drive));
      }
      addTuning("2: Feedforward Ramp (quick)", TuningCommands.feedforwardRamp(drive));
      addTuning("2: Steady-State Sweep (kS/kV)", TuningCommands.steadyStateSweep(drive));
      addTuning("2: Spin SysId (all four)", TuningCommands.spinSysIdFull(drive));
      addTuning("3: Spin Step Response", TuningCommands.spinStepResponse(drive));
      addTuning("3: Turn Step Response", TuningCommands.turnStepResponse(drive));
      addTuning("4: Measure Wheel Radius", TuningCommands.measureWheelRadius(drive));
      addTuning("5: Drive Square (odometry check)", TuningCommands.driveSquare(drive));
      addTuning("opt: Drive Step Response", TuningCommands.driveStepResponse(drive));
      addTuning("opt: Drive SysId (all four)", TuningCommands.driveSysIdFull(drive));
    }
  }

  private void addTuning(String name, Command command) {
    autoRoutines.add(new AutoRoutine(name, TUNING_GROUP, command));
  }

  /**
   * Publishes the operating modes the driver station offers, including one per autonomous routine.
   *
   * <p>The 2027 driver station is operating-mode driven: it lists the modes the robot publishes and
   * you pick one to enable. Nothing registers any by default -- {@code TimedRobot} has no
   * operating-mode code at all and the backend's registry starts empty -- so a robot that never
   * calls {@link RobotState#addOpMode} publishes an empty list and there is nothing to select.
   *
   * <p>Nothing limits a robot mode to a single entry, which is what replaces the dashboard chooser
   * here. {@code addOpMode} rejects a duplicate name within a robot mode and otherwise takes as
   * many autonomous entries as it is given, each with a group the driver station files it under. Its
   * javadoc describes exactly this use: in a match the selected operating mode "will indicate the
   * operating mode selected for auto before the match starts". Selection then costs no dashboard at
   * all, and AdvantageKit already records it as {@code /DriverStation/OpMode}, so it is captured and
   * replayed the same way the chooser's selection was.
   *
   * <p>Registration is static, so this does not require extending {@code OpModeRobot}. That class
   * adds automatic discovery of {@code OpMode} subclasses and a per-mode lifecycle; underneath, its
   * own {@code publishOpModes()} simply calls {@link RobotState#publishOpModes()} exactly as this
   * does. Registering here keeps {@code TimedRobot} dispatching through {@code teleopPeriodic()}
   * and friends as usual, because those follow the driver station's robot mode rather than the
   * operating mode itself.
   */
  private void publishOpModes() {
    RobotState.addOpMode(RobotMode.TELEOPERATED, "Teleop");
    for (AutoRoutine routine : autoRoutines) {
      RobotState.addOpMode(RobotMode.AUTONOMOUS, routine.name(), routine.group());
    }
    RobotState.addOpMode(RobotMode.UTILITY, "Utility");
    RobotState.publishOpModes();
  }

  /** What the driver station currently has selected, for the display. */
  private String selectedRoutineName() {
    return routineFor(autoRoutines, RobotState.getOpMode()).name();
  }

  /**
   * Finds the routine the driver station has selected, falling back to the first.
   *
   * <p>Separate and static so the fallback can be tested: {@link RobotState#getOpMode()} documents
   * that it "may return a string not in the list of options", and a teleop or utility mode is
   * selected often enough that the miss is the normal case rather than an error.
   *
   * @param routines every routine on offer, the first being the fallback
   * @param selectedName what the driver station reports as selected
   * @return the matching routine, or the first one
   */
  static AutoRoutine routineFor(List<AutoRoutine> routines, String selectedName) {
    for (AutoRoutine routine : routines) {
      if (routine.name().equals(selectedName)) {
        return routine;
      }
    }
    return routines.get(0);
  }

  /** The command to run in autonomous, from the operating mode selected on the driver station. */
  public Command getAutonomousCommand() {
    return routineFor(autoRoutines, RobotState.getOpMode()).command();
  }

  /** Puts the drive motors into brake or coast. */
  public void setMotorBrake(boolean brake) {
    drive.setBrakeMode(brake);
  }
}
