// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import java.util.function.DoubleConsumer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.driverstation.RobotState;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;

/**
 * Bring-up and tuning routines.
 *
 * <p>These exist because several numbers in {@link Constants} are inherited rather than measured:
 * the PID gains came from a YAGSL config that closed its loops in different units, the gear ratio
 * and wheel size look unusual, and the absolute encoder offsets were recorded against a different
 * encoder-reading convention. Each routine below turns one of those unknowns into a measurement.
 *
 * <p>All of them log to the {@code Tuning/} table, so the way to read a result is to run the routine
 * and then look at the plot in AdvantageScope rather than at a number on the dashboard. Gains
 * themselves are edited live through {@link TunableNumber} while a routine runs, so a tuning session
 * is: start the routine, watch the plot, drag the gain, watch again — with no redeploy in the loop.
 */
public final class TuningCommands {
  private TuningCommands() {}

  /**
   * Square-wave step response for the turn controllers, for tuning turn kP and kD.
   *
   * <p>Flips all four modules between two headings on a fixed period. Plot
   * {@code Tuning/TurnSetpointDeg} against each module's {@code Drive/&lt;name&gt;/TurnPositionDeg}:
   * too little kP and the module never reaches the setpoint before the next flip; too much and it
   * overshoots and rings. Raise kP until it just starts to overshoot, then back off or add a little
   * kD.
   *
   * <p>Safe to run with the robot on blocks or on the floor — the wheels are held at zero speed.
   *
   * @param drive the drivetrain
   * @return a command that steps until interrupted
   */
  public static Command turnStepResponse(Drive drive) {
    TunableNumber stepDegrees = new TunableNumber("Tuning/Turn/StepDegrees", 90.0);
    TunableNumber periodSecs = new TunableNumber("Tuning/Turn/StepPeriodSecs", 1.5);
    Timer timer = new Timer();

    return drive
        .run(
            coroutine -> {
              timer.restart();
              while (true) {
                // Alternate between +step and -step every period.
                boolean high = (long) (timer.get() / periodSecs.get()) % 2 == 0;
                double target = high ? stepDegrees.get() : -stepDegrees.get();
                drive.runTurnSetpoint(Rotation2d.fromDegrees(target));
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              timer.stop();
            })
        .named("Turn Step Response");
  }

  /**
   * Square-wave step response for the drive velocity controllers, for tuning drive kP.
   *
   * <p>Holds the modules straight ahead and alternates the commanded wheel speed between two values.
   * Plot {@code Tuning/DriveSetpointRadPerSec} against {@code Tuning/DriveMeasuredRadPerSec}: with
   * the feedforward correct the measured speed should already sit close to the setpoint, and kP is
   * only closing the remaining gap. If a large kP is needed to reach the setpoint at all, the
   * feedforward is wrong — re-run the SysId routines rather than fighting it with kP.
   *
   * <p><b>The robot will drive.</b> Put it on blocks, or give it room.
   *
   * @param drive the drivetrain
   * @return a command that steps until interrupted
   */
  public static Command driveStepResponse(Drive drive) {
    TunableNumber stepRadPerSec = new TunableNumber("Tuning/Drive/StepRadPerSec", 10.0);
    TunableNumber periodSecs = new TunableNumber("Tuning/Drive/StepPeriodSecs", 2.0);
    Timer timer = new Timer();

    return drive
        .run(
            coroutine -> {
              timer.restart();
              while (true) {
                boolean high = (long) (timer.get() / periodSecs.get()) % 2 == 0;
                drive.runDriveSetpoint(high ? stepRadPerSec.get() : 0.0);
                Logger.recordOutput(
                    "Tuning/DriveMeasuredRadPerSec", drive.getAverageWheelVelocityRadPerSec());
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              timer.stop();
            })
        .named("Drive Step Response");
  }

  /**
   * Drive kP step response run as a spin rather than a straight line.
   *
   * <p>Identical to {@link #driveStepResponse(Drive)} in what it measures — the drive controllers see
   * the same velocity setpoints and the same load — but the robot rotates on the spot instead of
   * travelling, so it never leaves its own footprint no matter how long you leave it running. This
   * is the one to use in a small space, and the one to prefer generally since the straight-line
   * version is open-ended.
   *
   * @param drive the drivetrain
   * @return a command that steps until interrupted
   */
  public static Command spinStepResponse(Drive drive) {
    TunableNumber stepRadPerSec = new TunableNumber("Tuning/Drive/StepRadPerSec", 10.0);
    TunableNumber periodSecs = new TunableNumber("Tuning/Drive/StepPeriodSecs", 2.0);
    Timer timer = new Timer();

    return drive
        .run(
            coroutine -> {
              timer.restart();
              while (true) {
                boolean high = (long) (timer.get() / periodSecs.get()) % 2 == 0;
                drive.runSpinSetpoint(high ? stepRadPerSec.get() : 0.0);
                Logger.recordOutput(
                    "Tuning/DriveMeasuredRadPerSec", drive.getAverageWheelVelocityRadPerSec());
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              timer.stop();
            })
        .named("Spin Step Response");
  }

  /**
   * A single slow voltage ramp that plots applied volts against measured speed, so kS and kV can be
   * read straight off the graph.
   *
   * <p>This measures the same two gains as {@link #spinSysIdFull(Drive)} and is the quick version of
   * that step: one run instead of four, and the answer is read in AdvantageScope rather than exported
   * to the SysId tool. Plot {@code Tuning/Feedforward/SpeedMetersPerSec} on the x axis against
   * {@code Tuning/Feedforward/Volts} on the y — the intercept is kS and the slope is kV. Use the
   * full SysId routine when you want its statistics and its recommended kP; use this when you just
   * want to see whether the numbers are sane.
   *
   * <p>Each module is plotted separately — {@code Tuning/Feedforward/&lt;Name&gt;/SpeedMetersPerSec}
   * — because kS and kV are held per corner. Four lines that lie on top of each other mean the
   * modules match; one line offset upward is a corner with extra static friction, which is exactly
   * what the 2026 measurement saw on front-right.
   *
   * <p>Spins rather than driving straight, and shares the ramp rate and timeout in
   * {@link Constants.SysId}, so it covers the same ~1.75 rotations as one quasistatic run. The
   * original version of this routine ramped to 6 V over 12 s, which on this drivetrain would have
   * been 29 m of travel.
   *
   * @param drive the drivetrain
   * @return a command that ramps, then stops itself
   */
  public static Command feedforwardRamp(Drive drive) {
    Timer timer = new Timer();
    return drive
        .run(
            coroutine -> {
              timer.restart();
              while (true) {
                double volts = timer.get() * Constants.SysId.RAMP_RATE_VOLTS_PER_SEC;
                drive.runCharacterizationSpin(volts);
                Logger.recordOutput("Tuning/Feedforward/Volts", volts);
                for (Module module : drive.getModules()) {
                  Logger.recordOutput(
                      "Tuning/Feedforward/" + module.getName() + "/SpeedMetersPerSec",
                      module.getVelocityMetersPerSec());
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              timer.stop();
              System.out.println("=== Feedforward ramp done ===");
              System.out.println(
                  "  Plot Tuning/Feedforward/<Module>/SpeedMetersPerSec on x against"
                      + " Tuning/Feedforward/Volts on y.");
              System.out.println(
                  "  Per module: intercept is that corner's kS, slope is its kV. Currently:");
              for (Module module : drive.getModules()) {
                System.out.printf(
                    "    %-11s kS %.4f V, kV %.5f V/(rad/s)%n",
                    module.getName(), module.getDriveKs(), module.getDriveKv());
              }
              System.out.println("  Set the measured values in Constants.ModuleConfig.");
            })
        .until(() -> timer.get() >= Constants.SysId.QUASISTATIC_TIMEOUT_SECS)
        .named("Feedforward Ramp");
  }

  /**
   * Declares the wheels to be pointing straight forward, zeroing every turn encoder there.
   *
   * <p>This is how the modules find themselves on a robot without absolute encoders. Straighten all
   * four wheels by hand — a straight edge along each side of the chassis is the usual way — then run
   * this. Nothing moves; it only changes what the modules believe.
   *
   * <p>Only acts while the robot is disabled. Re-zeroing mid-match would tell every module it is
   * pointing forward when it is not, and the robot would drive off in whatever direction the wheels
   * happened to be sitting.
   *
   * <p>Expect to need this often. The steering coasts once the brake timer expires after disable, so
   * pushing the robot around is usually enough to lose alignment.
   *
   * @param drive the drivetrain
   * @return a command that zeroes once and ends
   */
  public static Command zeroModules(Drive drive) {
    return Command.noRequirements(
            coroutine -> {
              if (!RobotState.isDisabled()) {
                System.out.println("Zero Modules ignored: only runs while disabled.");
                return;
              }
              drive.zeroModules();
              System.out.println(
                  "=== Modules zeroed. All four now read 0 deg; make sure they were straight. ===");
            })
        // Commands v3 has no enabled/disabled gate, so v2's ignoringDisable(true) has no
        // equivalent and needs none. The guard above is what keeps this disabled-only, and it was
        // always the real one -- ignoringDisable only ever granted permission to run.
        .named("Zero Modules");
  }

  /**
   * Reports the absolute encoder offsets needed to make the current physical module positions read
   * as zero.
   *
   * <p>Point all four wheels straight forward by hand — a straight edge along each side of the
   * chassis is the usual way — then run this. It logs, for each module, the offset that would put
   * that module's reported heading at zero in its current position. Copy the four numbers into
   * {@code Constants.ModuleConfig} and redeploy.
   *
   * <p>Worth doing before anything else on this robot: the offsets currently in {@code Constants}
   * were carried over from the YAGSL config, and YAGSL read the Thrifty encoders through its own
   * conversion. There is no reason to expect them to be right here.
   *
   * <p>This command moves nothing, so it is safe to run with the robot on the floor. It is marked to
   * run while disabled so the wheels can be positioned by hand.
   *
   * @param drive the drivetrain
   * @return a command that reports once and ends
   */
  public static Command reportEncoderOffsets(Drive drive) {
    return Command.noRequirements(
            coroutine -> {
              System.out.println("=== Absolute encoder offsets ===");
              System.out.println("Wheels must be pointing straight forward for these to be valid.");
              for (Module module : drive.getModules()) {
                // The module currently reports getAbsolutePosition() but should report zero, so the
                // configured offset needs to move by exactly that much.
                double correctionDeg = module.getAbsolutePosition().getDegrees();
                double newOffsetDeg =
                    wrapDegrees(module.getConfig().absoluteEncoderOffsetDegrees + correctionDeg);
                System.out.printf(
                    "  %-11s current reading %8.2f deg -> new offset %8.2f deg%n",
                    module.getName(), correctionDeg, newOffsetDeg);
                Logger.recordOutput("Tuning/Offsets/" + module.getName(), newOffsetDeg);
              }
              System.out.println("Copy these into Constants.ModuleConfig and redeploy.");
            })
        // Nothing moves, so there is nothing to gate on; see zeroModules for why v3 needs no
        // ignoringDisable.
        .named("Report Encoder Offsets");
  }

  private static double wrapDegrees(double degrees) {
    double wrapped = degrees % 360.0;
    return wrapped < 0 ? wrapped + 360.0 : wrapped;
  }

  /**
   * Measures the true wheel radius by driving a known number of wheel rotations in a straight line.
   *
   * <p>{@code Constants.Module.WHEEL_RADIUS} and {@code DRIVE_GEAR_RATIO} were both inherited and
   * both look unusual (a 2 in wheel on a 1.36:1 reduction). Together they set the conversion between
   * motor rotations and metres, so if either is wrong, every distance the robot believes is wrong by
   * the same factor — including odometry and any path following added later.
   *
   * <p>To use: mark the robot's starting position, run the routine, and measure the distance it
   * actually travelled with a tape. The routine logs the distance it *thinks* it travelled. The
   * corrected radius is {@code WHEEL_RADIUS * (measured / believed)}.
   *
   * <p><b>The robot will drive forward.</b> Give it several metres of clear space.
   *
   * @param drive the drivetrain
   * @return a command that drives, reports, and ends
   */
  public static Command measureWheelRadius(Drive drive) {
    TunableNumber wheelRadians = new TunableNumber("Tuning/WheelRadius/TargetWheelRadians", 40.0);
    TunableNumber velocityRadPerSec = new TunableNumber("Tuning/WheelRadius/SpeedRadPerSec", 8.0);
    double[] startRadians = new double[1];

    return drive
        .run(
            coroutine -> {
              startRadians[0] = drive.getCharacterizationPosition();
              while (true) {
                drive.runDriveSetpoint(velocityRadPerSec.get());
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              double travelledRadians = drive.getCharacterizationPosition() - startRadians[0];
              double believedMeters = travelledRadians * Constants.Module.WHEEL_RADIUS;
              System.out.println("=== Wheel radius check ===");
              System.out.printf("  wheel travel:      %.3f rad%n", travelledRadians);
              System.out.printf("  believed distance: %.4f m%n", believedMeters);
              System.out.println("  Measure the real distance, then set");
              System.out.printf(
                  "    WHEEL_RADIUS = %.6f * (measured_m / %.4f)%n",
                  Constants.Module.WHEEL_RADIUS, believedMeters);
              Logger.recordOutput("Tuning/WheelRadius/BelievedMeters", believedMeters);
            })
        .until(() -> drive.getCharacterizationPosition() - startRadians[0] >= wheelRadians.get())
        .named("Measure Wheel Radius");
  }

  /**
   * Builds the SysId routine for the drive motors.
   *
   * <p>This is the formal way to get kS, kV and kA — and SysId's analysis also recommends a kP,
   * which is the best starting point for {@link Constants.Module#DRIVE_KP}. The gains currently in
   * Constants came from a SysId run on the old YAGSL project; re-run this if the gear ratio or wheel
   * size turns out to be different from what was inherited, because those change the units the gains
   * are expressed in.
   *
   * <p>Run all four of the commands below in sequence, then load the resulting log into the SysId
   * tool. AdvantageKit records the routine's state to {@code Tuning/SysIdState}, which is what the
   * tool uses to split the log into the four tests.
   *
   * @param drive the drivetrain
   * @return the routine; call {@code quasistatic}/{@code dynamic} on it for the four test commands
   */
  public static SysIdRoutine driveSysIdRoutine(Drive drive) {
    return sysIdRoutine(drive, drive::runCharacterization, "Straight");
  }

  /**
   * The same routine with the modules pointed tangentially, so the robot spins in place.
   *
   * <p>This measures kS and kV exactly as well as the straight-line version — both are per-wheel
   * properties, and the wheels do the same work whether they follow a line or a circle — while
   * fitting in about a metre instead of three each way.
   *
   * <p>The one quantity it does not transfer is kA, which in a spin reflects the robot's rotational
   * inertia rather than its mass. That does not matter here: the drive feedforward is
   * {@code kS·sign(v) + kV·v}, with no kA term anywhere in the control path.
   *
   * @param drive the drivetrain
   * @return the routine
   */
  public static SysIdRoutine spinSysIdRoutine(Drive drive) {
    return sysIdRoutine(drive, drive::runCharacterizationSpin, "Spin");
  }

  private static SysIdRoutine sysIdRoutine(Drive drive, DoubleConsumer output, String name) {
    return new SysIdRoutine(drive, output, "Drive" + name);
  }

  /**
   * The four SysId voltage profiles, rebuilt on commands v3.
   *
   * <p>Commands v3 ships no SysId support -- {@code org.wpilib.command2.sysid.SysIdRoutine} exists
   * only in v2, and the two vendordeps refuse to coexist -- so the profiles are generated here
   * instead. Only the command plumbing was v2's; the part that matters to the analyser is the log,
   * and that is unchanged. The state strings come from {@link SysIdRoutineLog.State}, which lives
   * in wpilibj core rather than in either command framework, so {@code Tuning/SysIdState} still
   * reads {@code quasistatic-forward}, {@code dynamic-reverse} and so on, and the tool still splits
   * a log into the four tests exactly as before.
   *
   * <p>The profiles themselves are v2's, to the volt: a quasistatic test ramps at
   * {@link Constants.SysId#RAMP_RATE_VOLTS_PER_SEC} from zero, a dynamic test holds
   * {@link Constants.SysId#STEP_VOLTS} flat, reverse negates, and every test drops to zero volts
   * and writes {@code none} when it ends however it ends.
   *
   * <p>What is <em>not</em> carried over is v2's per-motor {@code SysIdRoutineLog} data callback,
   * which this project never supplied -- it passed {@code null} and relied on AdvantageKit logging
   * the drive signals itself, which it still does.
   */
  public static final class SysIdRoutine {
    /** Which way round to run a test. */
    public enum Direction {
      /** Positive voltage. */
      FORWARD,
      /** Negative voltage. */
      REVERSE
    }

    private final Drive drive;
    private final DoubleConsumer output;
    private final String name;

    private SysIdRoutine(Drive drive, DoubleConsumer output, String name) {
      this.drive = drive;
      this.output = output;
      this.name = name;
    }

    /**
     * A slow voltage ramp, for the gains that show up at steady state.
     *
     * @param direction which way to ramp
     * @return a command that ramps until the quasistatic timeout, then stops
     */
    public Command quasistatic(Direction direction) {
      double sign = direction == Direction.FORWARD ? 1.0 : -1.0;
      SysIdRoutineLog.State state =
          direction == Direction.FORWARD
              ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
              : SysIdRoutineLog.State.QUASISTATIC_REVERSE;
      Timer timer = new Timer();

      return test(
          coroutine -> {
            timer.restart();
            while (true) {
              output.accept(sign * timer.get() * Constants.SysId.RAMP_RATE_VOLTS_PER_SEC);
              Logger.recordOutput("Tuning/SysIdState", state.toString());
              coroutine.yield();
            }
          },
          () -> timer.get() >= Constants.SysId.QUASISTATIC_TIMEOUT_SECS,
          state);
    }

    /**
     * A voltage step held flat, for the gains that only show up under acceleration.
     *
     * @param direction which way to step
     * @return a command that holds the step until the dynamic timeout, then stops
     */
    public Command dynamic(Direction direction) {
      double sign = direction == Direction.FORWARD ? 1.0 : -1.0;
      SysIdRoutineLog.State state =
          direction == Direction.FORWARD
              ? SysIdRoutineLog.State.DYNAMIC_FORWARD
              : SysIdRoutineLog.State.DYNAMIC_REVERSE;
      Timer timer = new Timer();

      return test(
          coroutine -> {
            timer.restart();
            while (true) {
              output.accept(sign * Constants.SysId.STEP_VOLTS);
              Logger.recordOutput("Tuning/SysIdState", state.toString());
              coroutine.yield();
            }
          },
          // v2 took the dynamic timeout from a withTimeout() on the returned command rather than
          // from its Config, which set the quasistatic one. Same two numbers, applied the same way.
          () -> timer.get() >= Constants.SysId.DYNAMIC_TIMEOUT_SECS,
          state);
    }

    /**
     * The shape both tests share: drive the mechanism until the end condition, then stop it and
     * mark the log as running no test.
     *
     * <p>The stop goes in {@code whenCanceled} rather than in a {@code finally}, because v3 ends a
     * command by abandoning its coroutine rather than by unwinding it -- an {@code until} condition
     * cancels the command, and a {@code finally} in the body would never run. {@code whenCanceled}
     * is the hook that does, and since the body is an unbounded loop it is the only way out.
     */
    private Command test(
        java.util.function.Consumer<Coroutine> body,
        java.util.function.BooleanSupplier endCondition,
        SysIdRoutineLog.State state) {
      return drive
          .run(body)
          .whenCanceled(
              () -> {
                output.accept(0.0);
                drive.stop();
                Logger.recordOutput("Tuning/SysIdState", SysIdRoutineLog.State.NONE.toString());
              })
          .until(endCondition)
          .named("sysid-" + state + "-" + name);
    }
  }

  /**
   * Steady-state speed the feedforward predicts at a given voltage, in m/s.
   *
   * <p>Only as good as kS and kV, which are exactly what SysId is being run to re-measure — so treat
   * the distance predictions below as a planning figure, not a guarantee. The likely error is on the
   * safe side: if the drivetrain is geared down more than the config claims, it will be slower than
   * predicted and use less room.
   */
  private static double predictedSpeed(double volts) {
    return Math.max(
        0.0, (volts - Constants.Module.DRIVE_KS) / Constants.Module.DRIVE_KV_PER_METER_PER_SEC);
  }

  /** Distance a quasistatic ramp covers before its timeout, in metres. */
  private static double quasistaticDistance() {
    double rate = Constants.SysId.RAMP_RATE_VOLTS_PER_SEC;
    double timeout = Constants.SysId.QUASISTATIC_TIMEOUT_SECS;
    double startTime = Constants.Module.DRIVE_KS / rate;
    if (timeout <= startTime) {
      return 0.0;
    }
    // Integrate (rate*t - kS)/kV from the moment it breaks static friction to the timeout.
    return (rate * (timeout * timeout - startTime * startTime) / 2
            - Constants.Module.DRIVE_KS * (timeout - startTime))
        / Constants.Module.DRIVE_KV_PER_METER_PER_SEC;
  }

  /** Distance a dynamic step covers, in metres. Ignores the acceleration ramp, so it over-estimates. */
  private static double dynamicDistance() {
    return predictedSpeed(Constants.SysId.STEP_VOLTS) * Constants.SysId.DYNAMIC_TIMEOUT_SECS;
  }

  /**
   * The four SysId tests back to back, with a pause between each so the robot can be repositioned.
   *
   * <p><b>The robot will drive, in both directions.</b> Each quasistatic test ramps up slowly and
   * needs a few metres; the dynamic tests are shorter but more abrupt.
   *
   * @param drive the drivetrain
   * @return a command running all four tests in sequence
   */
  public static Command driveSysIdFull(Drive drive) {
    return sysIdFull(driveSysIdRoutine(drive), false);
  }

  /**
   * The four SysId tests run as spins rather than straight lines.
   *
   * <p><b>The robot will spin in place</b>, roughly one and three-quarter turns per run, alternating
   * direction. It needs about a metre of clear floor around it rather than three metres of runway.
   *
   * @param drive the drivetrain
   * @return a command running all four tests in sequence
   */
  public static Command spinSysIdFull(Drive drive) {
    return sysIdFull(spinSysIdRoutine(drive), true);
  }

  private static Command sysIdFull(SysIdRoutine routine, boolean spin) {
    return Command.sequence(
            Command.noRequirements(coroutine -> printSysIdBriefing(spin)).named("SysId Briefing"),
            routine.quasistatic(SysIdRoutine.Direction.FORWARD),
            pause(),
            routine.quasistatic(SysIdRoutine.Direction.REVERSE),
            pause(),
            // The dynamic runs reach full speed immediately and would otherwise cover far more
            // ground than the quasistatic ones, so they carry their own shorter timeout.
            routine.dynamic(SysIdRoutine.Direction.FORWARD),
            pause(),
            routine.dynamic(SysIdRoutine.Direction.REVERSE))
        .named(spin ? "Spin SysId" : "Drive SysId");
  }

  /** What the four runs are about to do, and how much floor they need. */
  private static void printSysIdBriefing(boolean spin) {
    System.out.println(spin ? "=== Drive SysId (spin) ===" : "=== Drive SysId ===");
    System.out.printf(
        "  quasistatic: %.1f V/s for %.1fs -> peak %.2f V, %.2f m/s, about %.1f m each%n",
        Constants.SysId.RAMP_RATE_VOLTS_PER_SEC,
        Constants.SysId.QUASISTATIC_TIMEOUT_SECS,
        Constants.SysId.RAMP_RATE_VOLTS_PER_SEC * Constants.SysId.QUASISTATIC_TIMEOUT_SECS,
        predictedSpeed(
            Constants.SysId.RAMP_RATE_VOLTS_PER_SEC * Constants.SysId.QUASISTATIC_TIMEOUT_SECS),
        quasistaticDistance());
    System.out.printf(
        "  dynamic:     %.1f V for %.1fs -> %.2f m/s, about %.1f m each%n",
        Constants.SysId.STEP_VOLTS,
        Constants.SysId.DYNAMIC_TIMEOUT_SECS,
        predictedSpeed(Constants.SysId.STEP_VOLTS),
        dynamicDistance());
    double longest = Math.max(quasistaticDistance(), dynamicDistance());
    if (spin) {
      System.out.printf(
          "  Spinning: %.2f rotations for the longest run. Clear about a metre around the%n"
              + "  robot; it stays over its own footprint.%n",
          longest / (2 * Math.PI * Constants.Drivebase.DRIVE_BASE_RADIUS));
    } else {
      System.out.printf(
          "  Longest single run is about %.1f m. The robot returns toward its start between%n"
              + "  forward and reverse pairs, so clear roughly that much in each direction.%n",
          longest);
    }
  }

  /** A second of stillness between tests, so the robot can be repositioned. */
  private static Command pause() {
    return Command.waitFor(Units.Seconds.of(1.0)).named("SysId Pause");
  }
  /**
   * Holds a series of fixed voltages long enough for the speed to settle at each, for measuring
   * drive kS and kV without an acceleration term contaminating them.
   *
   * <p>This exists because {@link #feedforwardRamp(Drive)} and the SysId quasistatic sweep cannot
   * actually separate kS from kA. Both ramp voltage at a constant rate, which on a linear plant
   * means constant acceleration for the whole run — so the kA column is collinear with the
   * constant term, and a three-parameter fit puts the entire acceleration contribution into the
   * intercept. That is not a subtle bias: at the ~16 rad/s^2 those runs held it is worth roughly
   * 0.05 V, against a kS of about 0.33.
   *
   * <p>Holding each voltage still until the speed stops changing removes the term entirely. What
   * is left is the steady-state line, which is what the feedforward actually needs, because the
   * feedforward's job is holding a speed rather than reaching one.
   *
   * <p>Spins in place like the ramp does, so it needs no more floor than the robot itself. The
   * defaults walk 0.15 V to 1.05 V in 0.1 V steps, 1.5 s each, which is about 15 s and tops out
   * near 180 deg/s of rotation. Every step is logged to {@code Tuning/SteadySweep/} and the fit is
   * printed at the end.
   *
   * <p>Two numbers come out and they mean different things. The fitted intercept is kS as the
   * feedforward uses it — the voltage the line says you need at zero speed. The breakaway voltage
   * is the lowest step that actually moved the wheel at all. Breakaway is the more physical of the
   * two and is usually a little higher, because a wheel that is already turning takes less voltage
   * to keep turning than a stopped one takes to start.
   *
   * @param drive the drivetrain
   * @return a command that sweeps, reports, and stops itself
   */
  public static Command steadyStateSweep(Drive drive) {
    TunableNumber startVolts = new TunableNumber("Tuning/SteadySweep/StartVolts", 0.15);
    TunableNumber endVolts = new TunableNumber("Tuning/SteadySweep/EndVolts", 1.05);
    TunableNumber stepVolts = new TunableNumber("Tuning/SteadySweep/StepVolts", 0.10);
    TunableNumber settleSecs = new TunableNumber("Tuning/SteadySweep/SettleSecs", 0.75);
    TunableNumber measureSecs = new TunableNumber("Tuning/SteadySweep/MeasureSecs", 0.75);

    Timer timer = new Timer();
    List<SweepStep> steps = new ArrayList<>();

    return drive
        .run(
            coroutine -> {
              steps.clear();
              timer.restart();
              while (true) {
                sweepTick(drive, steps, timer, startVolts, stepVolts, settleSecs, measureSecs);
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              timer.stop();
              reportSweep(drive, steps);
            })
        .until(
            () ->
                startVolts.get()
                        + (int) (timer.get() / (settleSecs.get() + measureSecs.get()))
                            * stepVolts.get()
                    > endVolts.get())
        .named("Steady-State Sweep");
  }

  /**
   * One loop of {@link #steadyStateSweep(Drive)}: hold this step's voltage, and once the wheel has
   * had time to settle, accumulate what speed it settled at.
   */
  private static void sweepTick(
      Drive drive,
      List<SweepStep> steps,
      Timer timer,
      TunableNumber startVolts,
      TunableNumber stepVolts,
      TunableNumber settleSecs,
      TunableNumber measureSecs) {
    double stepSecs = settleSecs.get() + measureSecs.get();
    int index = (int) (timer.get() / stepSecs);
    double volts = startVolts.get() + index * stepVolts.get();
    drive.runCharacterizationSpin(volts);
    Logger.recordOutput("Tuning/SteadySweep/Volts", volts);

    while (steps.size() <= index) {
      steps.add(new SweepStep(startVolts.get() + steps.size() * stepVolts.get()));
    }
    // Only the tail of each step counts: the first settleSecs is the wheel getting there.
    if (timer.get() - index * stepSecs >= settleSecs.get()) {
      Module[] modules = drive.getModules();
      for (int i = 0; i < modules.length; i++) {
        steps.get(index).add(i, modules[i].getVelocityMetersPerSec());
      }
    }
  }

  /** One voltage step of {@link #steadyStateSweep(Drive)}, accumulating settled speed per module. */
  private static final class SweepStep {
    private final double volts;
    private final double[] sum = new double[4];
    private final int[] count = new int[4];

    private SweepStep(double volts) {
      this.volts = volts;
    }

    private void add(int module, double speedMetersPerSec) {
      sum[module] += Math.abs(speedMetersPerSec);
      count[module]++;
    }

    private double speed(int module) {
      return count[module] == 0 ? 0.0 : sum[module] / count[module];
    }
  }

  /** Prints the sweep table and the per-module fit. */
  private static void reportSweep(Drive drive, List<SweepStep> steps) {
    Module[] modules = drive.getModules();
    System.out.println("=== Steady-state sweep done ===");
    System.out.printf("  %8s", "volts");
    for (Module module : modules) {
      System.out.printf("%14s", module.getName());
    }
    System.out.println("   (settled m/s)");
    for (SweepStep step : steps) {
      System.out.printf("  %8.3f", step.volts);
      for (int i = 0; i < modules.length; i++) {
        System.out.printf("%14.4f", step.speed(i));
      }
      System.out.println();
    }

    System.out.println("  Fit over the steps that moved:");
    double ksTotal = 0.0;
    double kvTotal = 0.0;
    int fitted = 0;
    for (int i = 0; i < modules.length; i++) {
      List<Double> xs = new ArrayList<>();
      List<Double> ys = new ArrayList<>();
      double breakaway = Double.NaN;
      for (SweepStep step : steps) {
        if (step.speed(i) > MOVING_METERS_PER_SEC) {
          if (Double.isNaN(breakaway)) {
            breakaway = step.volts;
          }
          xs.add(step.speed(i) / Constants.Module.WHEEL_RADIUS);
          ys.add(step.volts);
        }
      }
      if (xs.size() < 3) {
        System.out.printf("    %-11s too few moving steps to fit%n", modules[i].getName());
        continue;
      }
      double[] fit = fitLine(xs, ys);
      ksTotal += fit[0];
      kvTotal += fit[1];
      fitted++;
      System.out.printf(
          "    %-11s kS %.4f V, kV %.5f V/(rad/s), r2 %.5f, breakaway %.3f V%n",
          modules[i].getName(), fit[0], fit[1], fit[2], breakaway);
      Logger.recordOutput("Tuning/SteadySweep/" + modules[i].getName() + "/Ks", fit[0]);
      Logger.recordOutput("Tuning/SteadySweep/" + modules[i].getName() + "/Kv", fit[1]);
    }
    if (fitted > 0) {
      double ks = ksTotal / fitted;
      double kv = kvTotal / fitted;
      System.out.printf("  Mean: kS %.4f V, kV %.5f V/(rad/s)%n", ks, kv);
      System.out.printf(
          "  Set DRIVE_KS = %.4f and DRIVE_KV_PER_METER_PER_SEC = %.4f%n",
          ks, kv / Constants.Module.WHEEL_RADIUS);
      System.out.printf(
          "  Currently  DRIVE_KS = %.4f, DRIVE_KV = %.5f V/(rad/s)%n",
          Constants.Module.DRIVE_KS, Constants.Module.DRIVE_KV);
    }
  }

  /** Below this a module counts as not having broken loose, in m/s. */
  private static final double MOVING_METERS_PER_SEC = 0.01;

  /**
   * Least-squares fit of {@code y = intercept + slope * x}.
   *
   * @param xs the x values
   * @param ys the y values, same length
   * @return {@code {intercept, slope, rSquared}}
   */
  static double[] fitLine(List<Double> xs, List<Double> ys) {
    int n = xs.size();
    double meanX = xs.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    double meanY = ys.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    double sxx = 0.0;
    double sxy = 0.0;
    for (int i = 0; i < n; i++) {
      sxx += (xs.get(i) - meanX) * (xs.get(i) - meanX);
      sxy += (xs.get(i) - meanX) * (ys.get(i) - meanY);
    }
    double slope = sxx == 0.0 ? 0.0 : sxy / sxx;
    double intercept = meanY - slope * meanX;
    double residual = 0.0;
    double total = 0.0;
    for (int i = 0; i < n; i++) {
      double predicted = intercept + slope * xs.get(i);
      residual += (ys.get(i) - predicted) * (ys.get(i) - predicted);
      total += (ys.get(i) - meanY) * (ys.get(i) - meanY);
    }
    return new double[] {intercept, slope, total == 0.0 ? 1.0 : 1.0 - residual / total};
  }

  /**
   * Drives a closed square and stops where it started, for checking odometry against the floor.
   *
   * <p>Everything the drivetrain believes about distance and heading has been measured one term at
   * a time: wheel radius against a tape in a straight line, gear ratio on the bench, track radius
   * and gyro scale from a hand rotation. This is the test that exercises all of them at once, in
   * the motion the robot actually does — translating while holding a heading, four times, around a
   * loop that has to close.
   *
   * <p>Translates only; it never rotates. That is deliberate. A square driven by pivoting at each
   * corner mixes translation error and rotation error into one number and tells you nothing about
   * which is which. Holding one heading the whole way means a closure error is a translation
   * error, and the heading drift reported alongside it is a separate, independent read on the gyro.
   *
   * <p><b>The log cannot score this test.</b> Each leg ends when odometry says it has gone far
   * enough, so odometry closes the loop on its own terms and learns nothing from doing it. The
   * measurement is physical: mark the floor at a corner of the robot before starting, run it, and
   * measure where that corner ends up.
   *
   * <p>Not quite by construction, though, and the difference matters. Each leg stops within
   * {@link #CORNER_TOLERANCE_METERS} of its corner, so odometry finishes somewhere inside that
   * radius of the start rather than exactly on it — slop of the same order as the error being
   * looked for. The routine prints where odometry thinks it ended, as forward and left of the
   * start, and the real error is that vector minus the one measured on the floor.
   *
   * <p>The square is walked counter-clockwise, in the robot's own frame as it sits at the start:
   * forward, then left, then back, then right. It never rotates, so "forward" stays the direction
   * the robot was pointing when the routine began. That means the whole square lies forward and to
   * the left of the starting position, and nothing extends behind or to the right beyond the
   * robot's own body -- park it in the corner of the space with forward and left clear.
   *
   * <p>Sized to stay inside a small space. The default 0.75 m side plus the robot's own footprint
   * needs about 1.1 m square of clear floor, which leaves roughly a foot of margin on each side in
   * a 5.5 ft room. The required envelope is printed when the routine starts, so check it against
   * the room before letting it run with a larger side.
   *
   * @param drive the drivetrain
   * @return a command that drives the square, reports, and stops itself
   */
  public static Command driveSquare(Drive drive) {
    TunableNumber sideMeters = new TunableNumber("Tuning/Square/SideMeters", 0.75);
    TunableNumber speedMetersPerSec = new TunableNumber("Tuning/Square/SpeedMetersPerSec", 0.25);

    Pose2d[] start = new Pose2d[1];
    int[] leg = new int[1];
    double[] pathLength = new double[1];

    return drive
        .run(
            coroutine -> {
              start[0] = drive.getPose();
              leg[0] = 1;
              pathLength[0] = 0.0;
              System.out.println("=== Drive square starting ===");
              System.out.printf(
                  "  %.2f m sides need about %.2f m square of clear floor, robot included.%n",
                  sideMeters.get(), sideMeters.get() + ROBOT_ENVELOPE_METERS);
              System.out.println("  Counter-clockwise from here: forward, left, back, right.");
              System.out.println("  All of it lies forward and left of the robot, which never turns.");
              System.out.println("  Mark the floor at one corner of the robot first.");

              while (true) {
                squareTick(drive, start, leg, pathLength, sideMeters, speedMetersPerSec);
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              drive.stop();
              Pose2d end = drive.getPose();
              // In the frame the robot started in, so it reads as forward/left rather than as
              // field x/y that mean nothing to whoever is holding the tape measure.
              Translation2d residual =
                  end.getTranslation()
                      .minus(start[0].getTranslation())
                      .rotateBy(start[0].getRotation().unaryMinus());
              double headingDrift = end.getRotation().minus(start[0].getRotation()).getDegrees();
              System.out.println("=== Drive square done ===");
              System.out.printf(
                  "  legs completed:   %d of %d%n", Math.min(leg[0], SQUARE_CORNERS), SQUARE_CORNERS);
              System.out.printf("  path length:      %.3f m (%.1f in)%n", pathLength[0], pathLength[0] / 0.0254);
              System.out.printf("  heading drift:    %+.2f deg%n", headingDrift);
              // Not zero: each leg ends within CORNER_TOLERANCE_METERS of its corner, and that
              // slop is the same size as the error being measured, so it has to be subtracted
              // rather than waved away.
              System.out.printf(
                  "  odometry stopped: %+.2f in forward, %+.2f in left of the start%n",
                  residual.getX() / 0.0254, residual.getY() / 0.0254);
              System.out.println("  Measure the robot against its floor mark, forward and left of");
              System.out.println("  the start (negative for back and right). The odometry error is");
              System.out.println("  the vector above minus what you measured, over the path length.");
              System.out.println("  Short in every direction points at WHEEL_RADIUS; a lateral bias");
              System.out.println("  that survives it points at module pointing, not distance.");
              Logger.recordOutput("Tuning/Square/ResidualForwardMeters", residual.getX());
              Logger.recordOutput("Tuning/Square/ResidualLeftMeters", residual.getY());
              Logger.recordOutput("Tuning/Square/PathLengthMeters", pathLength[0]);
              Logger.recordOutput("Tuning/Square/HeadingDriftDeg", headingDrift);
            })
        .until(() -> leg[0] > SQUARE_CORNERS)
        .named("Drive Square");
  }

  /** One loop of {@link #driveSquare(Drive)}: head for the current corner, or advance to the next. */
  private static void squareTick(
      Drive drive,
      Pose2d[] start,
      int[] leg,
      double[] pathLength,
      TunableNumber sideMeters,
      TunableNumber speedMetersPerSec) {
    // Corners are laid out relative to where the robot was pointing when the routine
    // started, not along the field axes. Otherwise the shape of the square would depend
    // on whatever the gyro happened to read, and setting the test up would mean zeroing
    // the heading first and trusting it -- in a room this size, a heading that is off by
    // 45 degrees sends the robot diagonally into a wall.
    Translation2d target =
        start[0]
            .getTranslation()
            .plus(
                squareCorner(leg[0], sideMeters.get())
                    .rotateBy(start[0].getRotation()));

    Translation2d error = target.minus(drive.getPose().getTranslation());
    Logger.recordOutput("Tuning/Square/Leg", leg[0]);
    Logger.recordOutput("Tuning/Square/DistanceToCorner", error.getNorm());

    if (error.getNorm() < CORNER_TOLERANCE_METERS) {
      // Count the leg just finished before deciding whether to stop, or the last one
      // never gets counted and the reported path is a quarter short.
      leg[0]++;
      pathLength[0] += sideMeters.get();
      if (leg[0] > SQUARE_CORNERS) {
        drive.stop();
      }
      return;
    }

    // Full speed down the leg, easing off over the last stretch so it settles on the
    // corner instead of overshooting it and crabbing back.
    double speed =
        Math.min(speedMetersPerSec.get(), APPROACH_GAIN_PER_SEC * error.getNorm());
    speed = Math.max(speed, MIN_APPROACH_METERS_PER_SEC);
    Translation2d velocity = error.div(error.getNorm()).times(speed);

    // Hold the starting heading. Any rotation here would smear translation error and
    // heading error together, which is exactly what this test is built to keep apart.
    double headingError =
        start[0].getRotation().minus(drive.getPose().getRotation()).getRadians();
    double omega =
        // WPILib 2027 dropped MathUtil.clamp in favour of the JDK's own.
        Math.clamp(
            HEADING_GAIN_PER_SEC * headingError,
            -MAX_CORRECTION_RAD_PER_SEC,
            MAX_CORRECTION_RAD_PER_SEC);

    drive.runVelocity(
        new ChassisVelocities(velocity.getX(), velocity.getY(), omega)
            .toRobotRelative(drive.getRotation()));
  }


  /** Corners of the square, in order; the fourth returns to the start. */
  private static final int SQUARE_CORNERS = 4;


  /**
   * How close to a corner counts as having reached it, in metres.
   *
   * <p>This is slop the test cannot see past: the robot stops within it at every corner, so
   * odometry's own closure is this big before any real error is counted. At 0.02 it was the same
   * size as the closure being measured, which is no use. 0.01 is about 0.4 in, still reachable at
   * the approach speeds below without dithering.
   */
  private static final double CORNER_TOLERANCE_METERS = 0.01;

  /** Approach speed per metre of remaining distance, easing the robot onto each corner. */
  private static final double APPROACH_GAIN_PER_SEC = 1.5;

  /** Floor on the approach speed, so the last centimetres do not take forever. */
  private static final double MIN_APPROACH_METERS_PER_SEC = 0.05;

  /** Heading-hold gain, in rad/s per radian of error. */
  private static final double HEADING_GAIN_PER_SEC = 2.0;

  /** Cap on the heading-hold output, so a bad pose cannot spin the robot. */
  private static final double MAX_CORRECTION_RAD_PER_SEC = 0.5;

  /** Roughly how much floor the robot itself occupies, for the clearance note, in metres. */
  private static final double ROBOT_ENVELOPE_METERS = 0.35;

  /**
   * Offset from the start to corner {@code index} of a square walked +x, +y, -x, -y.
   *
   * <p>Corner 0 is the start, and corner {@value #SQUARE_CORNERS} is the start again — the leg
   * offsets have to sum to zero or the robot would not be asked to come home, and the test would
   * have nothing to measure.
   *
   * @param index which corner, 0 through {@value #SQUARE_CORNERS}
   * @param sideMeters length of one side
   * @return the offset from the starting translation
   */
  static Translation2d squareCorner(int index, double sideMeters) {
    // +x is the robot's forward and +y its left, so this walks counter-clockwise. The caller
    // rotates these into the field frame using the heading the routine started at.
    return switch (index) {
      case 1 -> new Translation2d(sideMeters, 0.0);
      case 2 -> new Translation2d(sideMeters, sideMeters);
      case 3 -> new Translation2d(0.0, sideMeters);
      // 0 is where the robot started and 4 is where it has to come back to: the same place.
      default -> Translation2d.ZERO;
    };
  }
}
