// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Checks that the angles the spin characterization points the modules at really do produce a pure
 * rotation, rather than a rotation plus some translation the test would silently smear into its
 * results.
 */
class SpinGeometryTest {
  private static final double EPSILON = 1e-9;

  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(Constants.Drivebase.MODULE_TRANSLATIONS);

  @Test
  void tangentAnglesProduceRotationWithoutTranslation() {
    double wheelSpeed = 1.0;
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < velocities.length; i++) {
      velocities[i] = new SwerveModuleVelocity(wheelSpeed, Drive.tangentAngle(i));
    }

    ChassisVelocities chassis = KINEMATICS.toChassisVelocities(velocities);

    assertEquals(0.0, chassis.vx, EPSILON, "spin should not translate along x");
    assertEquals(0.0, chassis.vy, EPSILON, "spin should not translate along y");
    assertTrue(chassis.omega > 0.0, "tangent angles should spin counter-clockwise");

    // Each wheel rides a circle of radius DRIVE_BASE_RADIUS, so the robot's angular rate is the
    // wheel speed divided by that radius. This is the conversion the routine's console output uses.
    assertEquals(
        wheelSpeed / Constants.Drivebase.DRIVE_BASE_RADIUS,
        chassis.omega,
        1e-9,
        "angular rate should be wheel speed over drive base radius");
  }

  @Test
  void tangentAnglesAreQuarterTurnFromModulePositions() {
    for (int i = 0; i < 4; i++) {
      double positionDeg = Constants.Drivebase.MODULE_TRANSLATIONS[i].getAngle().getDegrees();
      double tangentDeg = Drive.tangentAngle(i).getDegrees();
      double difference = Math.IEEEremainder(tangentDeg - positionDeg, 360.0);
      assertEquals(90.0, difference, 1e-9, "module " + i + " tangent should lead its position by 90 deg");
    }
  }
}
