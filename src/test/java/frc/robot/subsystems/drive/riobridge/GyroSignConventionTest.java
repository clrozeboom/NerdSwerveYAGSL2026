// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.riobridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The navX is clockwise-positive, WPILib is counter-clockwise-positive, and the RioBridge sends
 * {@code AHRS.getYaw()} across the wire without picking a side. Something has to flip it, and
 * {@link GyroIORioBridge} is the last place that can.
 *
 * <p>A sign error here is quiet in exactly the way that costs a match. Nothing throws, nothing
 * disconnects, the gyro looks alive and its magnitude is right; the robot just steers the wrong
 * way under field-oriented control the moment it is not facing forwards, and the logged pose
 * curves off the wrong side of the field. It took a spin step response to catch it -- a commanded
 * +1.191 rad/s logged as -1.404 rad/s while the robot visibly turned counter-clockwise.
 */
class GyroSignConventionTest {
  @Test
  void aClockwiseNavxReadingBecomesCounterClockwisePositive() {
    // The navX counts up as the robot turns clockwise. WPILib counts up the other way, so a navX
    // reading of +90 is a robot that has turned 90 degrees clockwise, which is -90 to WPILib.
    assertEquals(
        -90.0,
        GyroIORioBridge.yawFromNavx(90.0).getDegrees(),
        1e-9,
        "a navX yaw of +90 (clockwise) should report as -90 in WPILib's convention");

    assertEquals(
        45.0,
        GyroIORioBridge.yawFromNavx(-45.0).getDegrees(),
        1e-9,
        "a navX yaw of -45 (counter-clockwise) should report as +45");

    assertEquals(0.0, GyroIORioBridge.yawFromNavx(0.0).getDegrees(), 1e-9, "zero stays zero");
  }

  @Test
  void theSpinThatCaughtThisNowReportsPositive() {
    // The measured case: the robot was commanded to spin counter-clockwise at 1.191 rad/s, visibly
    // did so, and the navX -- counting the other way -- put that at -1.404 rad/s. That raw reading
    // is what went into the log unflipped, and it is what has to come back out positive.
    double navxYawRateDegPerSec = Math.toDegrees(-1.404);
    double reported = GyroIORioBridge.yawRateFromNavx(navxYawRateDegPerSec);

    assertTrue(
        reported > 0.0,
        "a counter-clockwise spin must report a positive yaw rate, not " + reported);
    assertEquals(1.404, reported, 1e-9, "magnitude should be untouched, only the sign flipped");
  }
}
