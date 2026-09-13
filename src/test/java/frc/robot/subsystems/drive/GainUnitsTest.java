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
  void driveGainsAreAlsoConvertedFromWheelRadiansToMotorRpm() {
    // REVLib alpha-7 dropped the SPARK's on-device conversion factors, so its velocity loop sees
    // error in motor RPM where it used to see wheel rad/s. That is a second conversion on top of
    // the volts-to-duty one, in the opposite direction, and it is exactly as invisible: the loop
    // runs either way, it is just wrong by the gear ratio and a factor of 60.
    double gain = Constants.Module.DRIVE_KP;
    double sparkUnits = ModuleIOSpark.driveGainToSparkUnits(gain);

    // One motor RPM of error is DRIVE_VELOCITY_FACTOR wheel rad/s, and the gain has to produce the
    // same duty cycle for it either way round.
    assertEquals(
        ModuleIOSpark.voltsPerErrorToDuty(gain) * ModuleIOSpark.DRIVE_VELOCITY_FACTOR,
        sparkUnits,
        1e-15,
        "the RPM gain should be the duty-cycle gain scaled by wheel rad/s per motor RPM");

    assertTrue(
        sparkUnits < ModuleIOSpark.voltsPerErrorToDuty(gain),
        "motor RPM is a smaller unit of error than wheel rad/s, so the gain must come down, not up");
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
