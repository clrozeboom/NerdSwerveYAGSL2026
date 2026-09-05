// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * {@link ModuleIOSpark} used to read its absolute position from a local {@code AnalogEncoder},
 * whose {@code get()} returns a 0-1 fraction of a turn. Now that it comes from the RioBridge
 * instead, {@code EncodersFrame.rawCounts()} arrives as raw 12-bit ADC counts (0-4095), so
 * something has to reproduce that same 0-1 fraction or every downstream offset in
 * {@code readAbsolutePosition()} would be scaled wrong without necessarily failing loudly.
 */
class RioBridgeEncoderConversionTest {
  @Test
  void endpointsAndMidpointMapToTheExpectedFraction() {
    assertEquals(
        0.0, ModuleIOSpark.adcCountToTurnFraction(0), 1e-12, "count 0 should be fraction 0.0");
    assertEquals(
        0.5,
        ModuleIOSpark.adcCountToTurnFraction(2048),
        1e-12,
        "count 2048 (half of 4096 codes) should be fraction 0.5");
    assertEquals(
        4095.0 / 4096.0,
        ModuleIOSpark.adcCountToTurnFraction(4095),
        1e-12,
        "the top code (4095) should fall just short of a full turn, not reach it");
  }
}
