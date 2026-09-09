// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * The turn loop cannot beat static friction with proportional gain: each module parks where the
 * voltage its error produces falls below its own breakaway, and the gain needed to shrink that
 * (about 12 on the stiffest corner) is above where this loop goes unstable — measured, at kP 8, as
 * modules spinning continuously instead of settling. The feedforward defeats friction without
 * raising loop gain, which is why it is the term that can actually be used.
 */
class TurnFeedforwardTest {
  private static final double KS = 0.43;
  private static final double TOL = Math.toRadians(1.5);

  @Test
  void pushesHardEnoughToBreakLooseOutsideTheBand() {
    // Just outside the band, the push must be the full breakaway voltage. Anything scaled down by
    // the error would be back to the proportional behaviour this exists to replace: too small to
    // move the module precisely when the module has nearly arrived.
    double justOutside = TOL * 1.01;
    assertEquals(
        KS,
        ModuleIOSpark.turnFeedforwardVolts(justOutside, KS, TOL),
        1e-12,
        "the term should apply full breakaway voltage the moment the module is outside the band");
    assertEquals(
        -KS,
        ModuleIOSpark.turnFeedforwardVolts(-justOutside, KS, TOL),
        1e-12,
        "and push the other way when the module is on the other side");
  }

  @Test
  void switchesOffInsideTheBandSoTheModuleCanSettle() {
    // This is what stops it hunting. A fixed push that never switches off drives past the setpoint,
    // gets pushed back, and oscillates indefinitely -- swapping a steady-state error for a
    // permanent one that also wears the gearbox.
    for (double errDeg : new double[] {0.0, 0.5, 1.0, 1.49}) {
      assertEquals(
          0.0,
          ModuleIOSpark.turnFeedforwardVolts(Math.toRadians(errDeg), KS, TOL),
          1e-12,
          "no push at " + errDeg + " deg: inside the band the module must be left alone");
    }
  }

  @Test
  void theConfiguredGainsCannotSaturateTheTurnController() {
    // The feedforward stacks on top of the proportional term. At the largest error the loop ever
    // sees -- half a turn, since the controller wraps -- the two together must still leave the
    // output somewhere the controller can modulate, rather than pinned to the rail by the
    // feedforward alone.
    double worstError = Math.PI;
    for (Constants.ModuleConfig c : Constants.ModuleConfig.values()) {
      double ff = ModuleIOSpark.turnFeedforwardVolts(worstError, c.turnKs, TOL);
      assertTrue(
          Math.abs(ff) < Constants.Module.NOMINAL_VOLTAGE / 2.0,
          c + " feedforward alone is a large fraction of the battery: " + ff + " V");
    }
  }

  @Test
  void everyCornerHasAMeasuredBreakawayVoltage() {
    // These are read off turn step response logs (the held voltage while parked), not guessed. A
    // zero would silently disable the term for that corner.
    for (Constants.ModuleConfig c : Constants.ModuleConfig.values()) {
      assertTrue(
          c.turnKs > 0.0 && c.turnKs < 2.0,
          c + " turnKs looks unmeasured or implausible: " + c.turnKs);
    }
  }
}
