// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import frc.robot.RobotContainer.AutoRoutine;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;

/**
 * Picking the autonomous routine from the driver station's operating mode.
 *
 * <p>The lookup is by name against a string the driver station supplies, and
 * {@code RobotState.getOpMode()} documents that it "may return a string not in the list of
 * options". That is not an edge case here: any time teleop or utility is the selected mode the
 * lookup misses, so the miss path runs far more often than the hit and has to land somewhere safe
 * rather than return null.
 */
class AutoRoutineSelectionTest {
  private static AutoRoutine routine(String name) {
    return new AutoRoutine(
        name, "Test", Command.noRequirements(coroutine -> {}).named(name));
  }

  private static final AutoRoutine DO_NOTHING = routine("Do Nothing");
  private static final AutoRoutine SQUARE = routine("5: Drive Square (odometry check)");
  private static final AutoRoutine SWEEP = routine("2: Steady-State Sweep (kS/kV)");
  private static final List<AutoRoutine> ROUTINES = List.of(DO_NOTHING, SQUARE, SWEEP);

  @Test
  void theSelectedNamePicksItsOwnRoutine() {
    assertSame(SQUARE, RobotContainer.routineFor(ROUTINES, "5: Drive Square (odometry check)"));
    assertSame(SWEEP, RobotContainer.routineFor(ROUTINES, "2: Steady-State Sweep (kS/kV)"));
  }

  @Test
  void anUnrecognisedModeFallsBackToTheFirstRoutine() {
    // "Teleop" is a real operating mode this robot publishes, and it is what getOpMode() reports
    // for most of a session. It must not select a routine that moves the robot.
    assertSame(DO_NOTHING, RobotContainer.routineFor(ROUTINES, "Teleop"));
    assertEquals(
        "Do Nothing",
        RobotContainer.routineFor(ROUTINES, "something the driver station made up").name(),
        "an unknown mode has to be inert, not the first routine that happens to drive");
  }

  @Test
  void theFallbackIsNeverNullAndMatchingIsExact() {
    // Guards against a lookup that gets clever with prefixes or case: two tuning routines share
    // the "2: " prefix, and picking the wrong one drives the robot instead of spinning it.
    assertSame(DO_NOTHING, RobotContainer.routineFor(ROUTINES, "2: "));
    assertSame(DO_NOTHING, RobotContainer.routineFor(ROUTINES, "5: drive square (odometry check)"));
  }
}
