// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * What the configured status periods cost on can_s0.
 *
 * <p>The individual periods in {@link Constants.Module} are not interesting on their own -- what
 * matters is the total, because the bus is the thing in short supply. Eight SPARK MAXes at the
 * firmware default put roughly 1450 frames/s on can_s0, of which REVLib surfaced 4-6%, and the
 * whole reason the periods are set explicitly is to get that number down. A period changed without
 * thinking about the sum would undo it silently, which is what this guards.
 *
 * <p>The frame accounting comes from a 2000-frame {@code candump} taken with the robot program
 * stopped: each controller emits two fast status frames plus two slow ones, and a CTRE PDP at
 * device 0 contributes about 296 frames/s that this code cannot change.
 */
class CanBusBudgetTest {
  /** Fast status frames per controller -- encoder position and velocity. */
  private static final int FAST_FRAMES_PER_CONTROLLER = 2;

  /** The two slow frames each controller sends regardless, measured at ~9 frames/s together. */
  private static final double SLOW_FRAMES_PER_SEC = 9.0;

  /** The CTRE PDP at device 0, which this code does not configure. */
  private static final double PDP_FRAMES_PER_SEC = 296.0;

  /** Measured on hardware with the stock periods, against 4-6% delivery. */
  private static final double MEASURED_DEFAULT_FRAMES_PER_SEC = 1450.0;

  private static double controllerFramesPerSec(int periodMs) {
    return FAST_FRAMES_PER_CONTROLLER * (1000.0 / periodMs) + SLOW_FRAMES_PER_SEC;
  }

  private static double projectedBusFramesPerSec() {
    return 4 * controllerFramesPerSec(Constants.Module.DRIVE_STATUS_PERIOD_MS)
        + 4 * controllerFramesPerSec(Constants.Module.TURN_STATUS_PERIOD_MS)
        + PDP_FRAMES_PER_SEC;
  }

  @Test
  void theConfiguredPeriodsMeaningfullyReduceBusTraffic() {
    double projected = projectedBusFramesPerSec();

    assertTrue(
        projected < 0.7 * MEASURED_DEFAULT_FRAMES_PER_SEC,
        "the point of setting these periods is to shed bus load; projected "
            + Math.round(projected)
            + " frames/s is not meaningfully below the measured default of "
            + Math.round(MEASURED_DEFAULT_FRAMES_PER_SEC));
  }

  @Test
  void theDriveEncoderIsNotAskedForFasterThanTheLoopReadsIt() {
    // Drive position and velocity are consumed once per robot cycle. Asking the controller for
    // them faster only spends bus on frames nothing looks at -- which is the mistake that got the
    // bus into this state.
    assertTrue(
        Constants.Module.DRIVE_STATUS_PERIOD_MS >= 20,
        "the robot loop is 20 ms, so a drive status period below it is wasted bus; got "
            + Constants.Module.DRIVE_STATUS_PERIOD_MS);
  }

  @Test
  void theTurnControllersAreCheaperThanTheDriveControllers() {
    // The steering loop closes on the RioBridge absolute encoder over can_s1, so what the turn
    // SPARK reports is a backlash diagnostic and a stale-encoder fallback. If that ever stops
    // being true this assertion should be revisited rather than deleted.
    assertTrue(
        Constants.Module.TURN_STATUS_PERIOD_MS > Constants.Module.DRIVE_STATUS_PERIOD_MS,
        "turn telemetry feeds no control loop and should not cost the same as drive telemetry");
  }
}
