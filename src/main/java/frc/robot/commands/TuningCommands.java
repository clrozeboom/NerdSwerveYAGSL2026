// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Rotation2d;
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

    return Commands.runEnd(
            () -> {
              // Alternate between +step and -step every period.
              boolean high = (long) (timer.get() / periodSecs.get()) % 2 == 0;
              double target = high ? stepDegrees.get() : -stepDegrees.get();
              drive.runTurnSetpoint(Rotation2d.fromDegrees(target));
            },
            () -> {
              drive.stop();
              timer.stop();
            },
            drive)
        .beforeStarting(timer::restart);
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

    return Commands.runEnd(
            () -> {
              boolean high = (long) (timer.get() / periodSecs.get()) % 2 == 0;
              drive.runDriveSetpoint(high ? stepRadPerSec.get() : 0.0);
              Logger.recordOutput(
                  "Tuning/DriveMeasuredRadPerSec", drive.getAverageWheelVelocityRadPerSec());
            },
            () -> {
              drive.stop();
              timer.stop();
            },
            drive)
        .beforeStarting(timer::restart);
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
    return Commands.runOnce(
            () -> {
              System.out.println("=== Absolute encoder offsets ===");
              System.out.println("Wheels must be pointing straight forward for these to be valid.");
              for (Module module : drive.getModules()) {
                // The module currently reports getAbsolutePosition() but should report zero, so the
                // configured offset needs to move by exactly that much.
                double correctionDeg = module.getAbsolutePosition().getDegrees();
                double newOffsetDeg =
                    wrapDegrees(currentOffsetOf(module.getName()) + correctionDeg);
                System.out.printf(
                    "  %-11s current reading %8.2f deg -> new offset %8.2f deg%n",
                    module.getName(), correctionDeg, newOffsetDeg);
                Logger.recordOutput("Tuning/Offsets/" + module.getName(), newOffsetDeg);
              }
              System.out.println("Copy these into Constants.ModuleConfig and redeploy.");
            })
        // No motion, and the wheels have to be positioned by hand, so let it run disabled.
        .ignoringDisable(true);
  }

  /** The offset currently compiled in for a module, by name. */
  private static double currentOffsetOf(String moduleName) {
    for (Constants.ModuleConfig config : Constants.ModuleConfig.ORDERED) {
      if (displayName(config).equals(moduleName)) {
        return config.absoluteEncoderOffsetDegrees;
      }
    }
    return 0.0;
  }

  /** Maps the enum's SCREAMING_CASE onto the CamelCase names the modules log under. */
  private static String displayName(Constants.ModuleConfig config) {
    StringBuilder out = new StringBuilder();
    for (String part : config.name().split("_")) {
      out.append(part.charAt(0)).append(part.substring(1).toLowerCase());
    }
    return out.toString();
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

    return Commands.runEnd(
            () -> drive.runDriveSetpoint(velocityRadPerSec.get()),
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
            },
            drive)
        .beforeStarting(() -> startRadians[0] = drive.getCharacterizationPosition())
        .until(
            () ->
                drive.getCharacterizationPosition() - startRadians[0] >= wheelRadians.get());
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
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.per(Units.Second).of(Constants.SysId.RAMP_RATE_VOLTS_PER_SEC),
            Units.Volts.of(Constants.SysId.STEP_VOLTS),
            Units.Seconds.of(Constants.SysId.QUASISTATIC_TIMEOUT_SECS),
            state -> Logger.recordOutput("Tuning/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            voltage -> drive.runCharacterization(voltage.in(Units.Volts)), null, drive));
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
    SysIdRoutine routine = driveSysIdRoutine(drive);
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              System.out.println("=== Drive SysId ===");
              System.out.printf(
                  "  quasistatic: %.1f V/s for %.1fs -> peak %.2f V, %.2f m/s, about %.1f m each%n",
                  Constants.SysId.RAMP_RATE_VOLTS_PER_SEC,
                  Constants.SysId.QUASISTATIC_TIMEOUT_SECS,
                  Constants.SysId.RAMP_RATE_VOLTS_PER_SEC
                      * Constants.SysId.QUASISTATIC_TIMEOUT_SECS,
                  predictedSpeed(
                      Constants.SysId.RAMP_RATE_VOLTS_PER_SEC
                          * Constants.SysId.QUASISTATIC_TIMEOUT_SECS),
                  quasistaticDistance());
              System.out.printf(
                  "  dynamic:     %.1f V for %.1fs -> %.2f m/s, about %.1f m each%n",
                  Constants.SysId.STEP_VOLTS,
                  Constants.SysId.DYNAMIC_TIMEOUT_SECS,
                  predictedSpeed(Constants.SysId.STEP_VOLTS),
                  dynamicDistance());
              System.out.printf(
                  "  Longest single run is about %.1f m. The robot returns toward its start between%n"
                      + "  forward and reverse pairs, so clear roughly that much in each direction.%n",
                  Math.max(quasistaticDistance(), dynamicDistance()));
            }),
        routine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        routine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        // The Config timeout applies to both test types, so cap the dynamic runs separately —
        // they reach full speed immediately and would otherwise cover far more ground.
        routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .withTimeout(Constants.SysId.DYNAMIC_TIMEOUT_SECS),
        Commands.waitSeconds(1.0),
        routine
            .dynamic(SysIdRoutine.Direction.kReverse)
            .withTimeout(Constants.SysId.DYNAMIC_TIMEOUT_SECS));
  }
}
