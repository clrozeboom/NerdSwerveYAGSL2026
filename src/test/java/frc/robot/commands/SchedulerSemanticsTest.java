// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * The two commands v3 behaviours every routine in {@link TuningCommands} is built on.
 *
 * <p>Both concern how a command <i>ends</i>, which on this robot means how the motors get stopped.
 * Every tuning routine is an unbounded loop that drives the drivetrain, and each one hands its
 * {@code drive.stop()} to {@code whenCanceled}. Under commands v2 that cleanup lived in
 * {@code runEnd}'s end callback and ran on every exit path, so it was hard to get wrong. Under v3
 * it is not: the scheduler ends a command by abandoning its coroutine rather than unwinding it, so
 * a {@code finally} inside the body never executes and {@code whenCanceled} is the only hook that
 * does. If either assertion below stops holding, the robot keeps driving at the end of a routine
 * rather than stopping -- which is why this is pinned rather than assumed.
 */
class SchedulerSemanticsTest {
  /** A mechanism with no hardware behind it, purely to give two commands something to contend for. */
  private static final class FakeMechanism implements Mechanism {
    private final Scheduler scheduler;

    FakeMechanism(Scheduler scheduler) {
      this.scheduler = scheduler;
    }

    @Override
    public Scheduler getRegisteredScheduler() {
      return scheduler;
    }
  }

  @Test
  void anUntilConditionRunsTheEndCallbackExactlyOnce() {
    // How feedforwardRamp, measureWheelRadius, steadyStateSweep, driveSquare and both SysId tests
    // all finish: the body loops forever and an end condition stops it.
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    int[] loops = {0};
    int[] ended = {0};
    boolean[] done = {false};

    scheduler.schedule(
        Command.noRequirements(
                coroutine -> {
                  while (true) {
                    loops[0]++;
                    coroutine.yield();
                  }
                })
            .whenCanceled(() -> ended[0]++)
            .until(() -> done[0])
            .named("until-probe"));

    for (int i = 0; i < 3; i++) {
      scheduler.run();
    }
    assertTrue(loops[0] > 0, "the body should have been running, got " + loops[0] + " loops");
    assertEquals(0, ended[0], "nothing should have stopped the motors while the condition is false");

    done[0] = true;
    scheduler.run();
    int loopsAtEnd = loops[0];
    scheduler.run();

    assertEquals(1, ended[0], "the end callback has to fire exactly once when the condition trips");
    assertEquals(loopsAtEnd, loops[0], "and the body must not keep running after it");
  }

  @Test
  void periodicSideloadsRunBeforeCommandBodiesEveryLoop() {
    // Why Drive.periodic() can still be the input snapshot after losing its framework hook.
    // Commands v2 guaranteed the ordering by calling every subsystem's periodic() at the top of
    // CommandScheduler.run(); v3 has no subsystem hook, so RobotContainer registers it as a
    // sideload instead and relies on sideloads running first. If that ordering inverted, commands
    // would act on inputs read the previous loop.
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    List<String> order = new ArrayList<>();

    scheduler.addPeriodic(() -> order.add("periodic"));
    scheduler.schedule(
        Command.noRequirements(
                coroutine -> {
                  while (true) {
                    order.add("command");
                    coroutine.yield();
                  }
                })
            .named("ordering-probe"));

    scheduler.run();
    scheduler.run();
    scheduler.run();

    assertEquals(
        List.of("periodic", "command", "periodic", "command", "periodic", "command"),
        order,
        "inputs must be read before any command acts on them, every loop");
  }

  @Test
  void theBodyRunsBeforeTheUntilConditionIsFirstPolled() {
    // measureWheelRadius depends on this. Its body captures the starting wheel position on its
    // first run and its end condition measures travel against that baseline, so if the condition
    // were polled first it would compare against zero, read the whole absolute position as
    // distance already travelled, and end the routine before the robot moved. Under v2 the
    // ordering was explicit -- beforeStarting() was a separate command earlier in a sequence.
    // Under v3 until() is a race against a waitUntil command, and the ordering is the framework's
    // to decide, so it gets pinned here.
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    double[] baseline = {0.0};
    double position = 1000.0; // far past the target if the baseline is still zero
    int[] loops = {0};

    scheduler.schedule(
        Command.noRequirements(
                coroutine -> {
                  baseline[0] = position;
                  while (true) {
                    loops[0]++;
                    coroutine.yield();
                  }
                })
            .until(() -> position - baseline[0] >= 40.0)
            .named("init-order-probe"));

    scheduler.run();
    scheduler.run();
    scheduler.run();

    assertTrue(
        loops[0] >= 2,
        "the routine ended immediately, so until() saw the baseline before the body set it; loops="
            + loops[0]);
  }

  @Test
  void beingDisplacedByAnotherCommandAlsoRunsTheEndCallback() {
    // How the three step-response routines finish. They have no end condition at all -- they run
    // until something else claims the drivetrain, which on the robot is the driver's default
    // command or the next routine off the chooser.
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    FakeMechanism mechanism = new FakeMechanism(scheduler);
    int[] ended = {0};

    Command routine =
        mechanism
            .run(
                coroutine -> {
                  while (true) {
                    coroutine.yield();
                  }
                })
            .whenCanceled(() -> ended[0]++)
            .named("displaced-probe");

    scheduler.schedule(routine);
    scheduler.run();
    assertEquals(0, ended[0], "still the only command wanting the mechanism");

    scheduler.schedule(mechanism.run(coroutine -> coroutine.park()).named("usurper"));
    scheduler.run();

    assertEquals(1, ended[0], "losing the mechanism has to stop the motors, same as v2's runEnd");
  }
}
