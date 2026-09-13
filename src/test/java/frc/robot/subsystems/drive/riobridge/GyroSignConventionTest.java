// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.riobridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Two things have to happen to the navX's yaw before anything downstream can use it, and both are
 * the kind of error that stays quiet until it costs a match.
 *
 * <p>The navX is clockwise-positive where WPILib is counter-clockwise-positive, and the RioBridge
 * forwards {@code AHRS.getYaw()} across the wire without picking a side. A sign error there never
 * throws and never disconnects -- the gyro looks alive and its magnitude is right, and the robot
 * simply steers the wrong way under field-oriented control the moment it is not facing forwards.
 * It took a spin step response to catch: a commanded +1.191 rad/s logged as -1.404 while the robot
 * visibly turned counter-clockwise.
 *
 * <p>This particular navX2 also reads about 10% long, and keeps doing so after a recalibration.
 * That one survived even longer because it is invisible in any single measurement: yaw looks
 * smooth, drifts barely at all, and quantises exactly to the protocol's 0.01 degree. It only shows
 * up when rotation is compared against something else -- wheel travel, or a tape.
 */
class GyroSignConventionTest {
  @Test
  void aClockwiseNavxReadingBecomesCounterClockwisePositive() {
    // The navX counts up as the robot turns clockwise. WPILib counts up the other way, so a navX
    // reading of +90 is a robot that has turned 90 degrees clockwise, which is negative to WPILib.
    assertTrue(
        GyroIORioBridge.yawFromNavx(90.0).getDegrees() < 0.0,
        "a clockwise navX reading must report as a negative WPILib rotation");
    assertTrue(
        GyroIORioBridge.yawFromNavx(-45.0).getDegrees() > 0.0,
        "a counter-clockwise navX reading must report as a positive WPILib rotation");

    assertEquals(0.0, GyroIORioBridge.yawFromNavx(0.0).getDegrees(), 1e-9, "zero stays zero");
  }

  @Test
  void theMeasuredFullTurnComesBackAsThreeSixty() {
    // The measurement the scale factor is taken from: one hand-turned revolution logged 396.31
    // degrees of navX yaw. Dividing that back out has to land on the turn that actually happened.
    // Checked a quarter at a time, because Rotation2d is an orientation rather than a winding
    // count and folds a whole revolution down to zero -- which would make the obvious assertion
    // here pass against almost anything.
    assertEquals(
        -90.0,
        GyroIORioBridge.yawFromNavx(396.31 / 4.0).getDegrees(),
        1e-6,
        "a quarter of the navX's 396.31 degrees was a real quarter turn and should report as one");
  }

  @Test
  void theSpinThatCaughtTheSignNowReportsPositive() {
    // The robot was commanded to spin counter-clockwise and visibly did so, while the navX --
    // counting the other way -- called it negative. Signs must now agree with the command.
    double reported = GyroIORioBridge.yawRateFromNavx(Math.toDegrees(-1.404));

    assertTrue(
        reported > 0.0, "a counter-clockwise spin must report a positive yaw rate, not " + reported);
    assertEquals(
        1.404 / GyroIORioBridge.YAW_SCALE,
        reported,
        1e-9,
        "the rate carries the same 10% over-reading as the angle and is scaled the same way");
  }

  @Test
  void unwrappingSurvivesTheNavxRollingOverAtOneEighty() {
    // Why this class integrates yaw instead of scaling what arrives: past half a turn the navX
    // wraps, and scaling a wrapped reading silently reports the wrong angle. Walk 400 degrees of
    // real rotation past the seam in steps the navX would actually deliver.
    double continuous = GyroIORioBridge.unwrap(0.0, Double.NaN, 0.0);
    double previous = 0.0;
    for (double travelled = 10.0; travelled <= 400.0; travelled += 10.0) {
      // What the navX puts on the wire: the true angle, 10% long, wrapped into +/-180.
      double raw = travelled * GyroIORioBridge.YAW_SCALE;
      raw = ((raw + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
      continuous = GyroIORioBridge.unwrap(continuous, previous, raw);
      previous = raw;
    }

    assertEquals(
        400.0 * GyroIORioBridge.YAW_SCALE,
        continuous,
        1e-6,
        "unwrapping should rebuild the full 400 degrees the navX reported, wraps and all");
    // 400 degrees of travel leaves the robot pointing 40 degrees past where it started, and
    // clockwise, so WPILib calls that -40.
    assertEquals(
        -40.0,
        GyroIORioBridge.yawFromNavx(continuous).getDegrees(),
        1e-6,
        "and scaling the continuous angle should recover the 400 degrees actually travelled");
    // The trap this guards: scaling what arrived on the wire instead of the running total. The
    // final wrapped reading is nowhere near the right answer on its own.
    double lastWrapped = ((previous + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    assertTrue(
        Math.abs(GyroIORioBridge.yawFromNavx(lastWrapped).getDegrees() - (-40.0)) > 10.0,
        "scaling the wrapped reading directly should visibly disagree, or this test proves nothing");
  }
}
