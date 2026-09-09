// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * A velocity loop on a drivetrain with real static friction has a floor underneath it that a
 * textbook proportional loop does not: below {@link Constants.Module#DRIVE_KS} the motor makes no
 * useful torque at all, so any command under that voltage is the same command as zero.
 *
 * <p>That turns an ordinary overshoot into a limit cycle. At setpoint {@code v} the feedforward
 * sits at {@code kS + kV * v}, which clears breakaway by only {@code kV * v}. A proportional term
 * eats that margin at an error of {@code kV * v / kP}; past it the drive is commanded below
 * breakaway, the wheel coasts down instead of being trimmed, the error reverses, and the command
 * slams back over the floor. The loop stops being proportional and starts bang-banging.
 *
 * <p>This is not hypothetical. The spin step response at {@code DRIVE_KP = 0.2} spent 40% of its
 * settled time commanded below kS and surged between 2 and 23 rad/s against a 10 rad/s setpoint at
 * 2.8 Hz, on all four modules at once. None of it was visible in the SPARK's filtered velocity
 * signal, which reported a mean of 10.07 rad/s and looked like a clean tune; it only showed up in
 * raw encoder position. Simulation cannot catch this either, since {@link ModuleIOSim} has to be
 * given a friction model before kS has anything to cancel.
 *
 * <p>The requirement below is that an overspeed as large as top speed itself should not be enough
 * to cancel the feedforward margin. That reduces to {@code kP <= kV}, which is worth stating
 * plainly: on this drivetrain proportional is for trimming, and the feedforward does the driving.
 *
 * <p>It bounds the gain, it does not make the loop sound at any speed. The margin is proportional
 * to the setpoint, so it vanishes as the setpoint approaches zero no matter how small kP is —
 * crawling speeds are stiction's territory and always will be. That is what
 * {@link Constants.Module#DRIVE_CLOSED_LOOP_RAMP_RATE} is for.
 */
class DriveStictionMarginTest {
  /** Wheel speed, in rad/s, at the fastest the drivebase is allowed to ask for. */
  private static double maxWheelRadPerSec() {
    return Constants.Drivebase.MAX_LINEAR_SPEED / Constants.Module.WHEEL_RADIUS;
  }

  /**
   * The velocity error at which the proportional term cancels the whole feedforward margin above
   * static friction, leaving the drive commanded below breakaway.
   */
  private static double stictionFloorError(double kP, double kV, double setpointRadPerSec) {
    return kP <= 0.0 ? Double.POSITIVE_INFINITY : (kV * Math.abs(setpointRadPerSec)) / kP;
  }

  @Test
  void theProportionalTermCannotCancelTheFeedforwardMargin() {
    double topSpeed = maxWheelRadPerSec();
    double floorError =
        stictionFloorError(Constants.Module.DRIVE_KP, Constants.Module.DRIVE_KV, topSpeed);

    assertTrue(
        floorError >= topSpeed,
        "at top speed ("
            + topSpeed
            + " rad/s) an overspeed of only "
            + floorError
            + " rad/s drives the command below kS; kP "
            + Constants.Module.DRIVE_KP
            + " must not exceed kV "
            + Constants.Module.DRIVE_KV);
  }

  @Test
  void theGainThatLimitCycledOnHardwareWouldBeRejected() {
    // Guards the check above from being vacuously true: the gain that actually produced the 2.8 Hz
    // surge has to fail it, or the test is not measuring anything.
    double topSpeed = maxWheelRadPerSec();
    double floorError = stictionFloorError(0.2, Constants.Module.DRIVE_KV, topSpeed);

    assertTrue(
        floorError < topSpeed,
        "kP 0.2 is the gain that limit-cycled on hardware and should not satisfy the margin check");
  }
}
