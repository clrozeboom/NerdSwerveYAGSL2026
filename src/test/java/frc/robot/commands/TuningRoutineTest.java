// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * The arithmetic behind the two routines that produce a number rather than a plot.
 *
 * <p>Neither can be checked by running it: one reports gains that only mean something against real
 * hardware, and the other closes its own loop by construction and has to be scored with a tape.
 * What can be checked is that the fit is a fit and the square is a square.
 */
class TuningRoutineTest {
  private static final double SIDE = 0.75;

  @Test
  void theFitRecoversAnExactLine() {
    // kS 0.33 V plus kV 0.0266 V per rad/s, sampled where the sweep samples it.
    List<Double> speeds = List.of(5.0, 10.0, 15.0, 20.0, 25.0, 30.0);
    List<Double> volts = speeds.stream().map(v -> 0.33 + 0.0266 * v).toList();

    double[] fit = TuningCommands.fitLine(speeds, volts);

    assertEquals(0.33, fit[0], 1e-9, "intercept is kS");
    assertEquals(0.0266, fit[1], 1e-12, "slope is kV");
    assertEquals(1.0, fit[2], 1e-9, "a noiseless line should fit perfectly");
  }

  @Test
  void theFitReportsHowWellItFits() {
    // The same line with one point pulled well off it. r-squared has to drop, or it is not
    // reporting anything and a bad sweep would read as a good one.
    List<Double> speeds = List.of(5.0, 10.0, 15.0, 20.0, 25.0, 30.0);
    List<Double> volts = List.of(0.463, 0.596, 0.729, 0.862, 0.995, 2.500);

    double[] fit = TuningCommands.fitLine(speeds, volts);

    assertTrue(
        fit[2] < 0.95, "an obvious outlier should show up as a poor r-squared, got " + fit[2]);
  }

  @Test
  void theSquareCloses() {
    // The whole point of the routine is that the robot is asked to come back to where it started.
    // If the offsets did not sum to zero there would be nothing to measure against the floor.
    assertEquals(
        Translation2d.kZero,
        TuningCommands.squareCorner(0, SIDE),
        "the robot starts at the origin of its own square");
    assertEquals(
        Translation2d.kZero,
        TuningCommands.squareCorner(4, SIDE),
        "and the last corner has to be the first one again");
  }

  @Test
  void theSquareIsSquareAndFitsTheRoom() {
    Translation2d[] corners = {
      TuningCommands.squareCorner(0, SIDE),
      TuningCommands.squareCorner(1, SIDE),
      TuningCommands.squareCorner(2, SIDE),
      TuningCommands.squareCorner(3, SIDE)
    };

    for (int i = 0; i < corners.length; i++) {
      Translation2d leg = corners[(i + 1) % corners.length].minus(corners[i]);
      assertEquals(SIDE, leg.getNorm(), 1e-9, "leg " + i + " should be one side long");
    }

    // The path the robot's centre traces spans exactly one side in each axis, so the floor it
    // needs is that plus its own footprint. The default was chosen for a 5.5 ft room and should
    // leave real margin there rather than only just fitting.
    double spanX = Arrays.stream(corners).mapToDouble(Translation2d::getX).max().orElseThrow();
    double spanY = Arrays.stream(corners).mapToDouble(Translation2d::getY).max().orElseThrow();
    assertEquals(SIDE, spanX, 1e-9, "the path should be one side wide");
    assertEquals(SIDE, spanY, 1e-9, "and one side deep");

    double envelope = SIDE + 0.35;
    assertTrue(
        envelope < Units.feetToMeters(5.5) - 0.3,
        "the default side plus the robot should leave real margin in 5.5 ft, needs " + envelope);
  }
}
