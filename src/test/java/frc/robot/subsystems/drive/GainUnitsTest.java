// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * The PID gains in {@link Constants.Module} are stated in volts per unit of error, because that is
 * what {@link ModuleIOSim} applies directly. A SPARK MAX closed loop works in duty cycle instead,
 * so {@link ModuleIOSpark} has to convert.
 *
 * <p>Getting this wrong is not a subtle regression: passing a volts-shaped gain straight to a SPARK
 * makes it {@link Constants.Module#NOMINAL_VOLTAGE} times too aggressive, which on a real robot is
 * a saturated controller on a drivetrain, not a slightly-off tune. It is also invisible in
 * simulation, since nothing in the sim path goes near this conversion.
 */
class GainUnitsTest {
  @Test
  void sparkGainsAreConvertedFromVoltsToDutyCycle() {
    // A gain of NOMINAL_VOLTAGE volts per unit error is, by definition, full output per unit error.
    assertEquals(
        1.0,
        ModuleIOSpark.voltsPerErrorToDuty(Constants.Module.NOMINAL_VOLTAGE),
        1e-12,
        "a gain of one full battery per unit of error should be duty cycle 1.0");

    assertEquals(
        Constants.Module.DRIVE_KP / 12.0,
        ModuleIOSpark.voltsPerErrorToDuty(Constants.Module.DRIVE_KP),
        1e-12,
        "drive kP should be scaled down by the nominal voltage, not passed through");
  }

  @Test
  void theConfiguredDriveGainDoesNotSaturateTheController() {
    // The worst error the velocity loop can see is a full-speed reversal: commanded one way while
    // travelling the other. Even then the proportional term alone should not peg the output, or
    // there is no headroom left for the feedforward that does most of the work.
    double maxWheelRadPerSec = Constants.Drivebase.MAX_LINEAR_SPEED / Constants.Module.WHEEL_RADIUS;
    double worstError = 2.0 * maxWheelRadPerSec;
    double duty = ModuleIOSpark.voltsPerErrorToDuty(Constants.Module.DRIVE_KP) * worstError;

    assertTrue(
        duty < 1.0,
        "drive kP saturates the SPARK on a full-speed reversal: duty would be " + duty);
  }
}
