// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * This robot has 7-10 degrees of backlash between each turn motor and its module, so the motor's
 * own encoder and the absolute encoder genuinely disagree about where the module is pointing. Every
 * heading decision has to come from the module, not the motor: it is the wheel that has to end up
 * facing the right way.
 *
 * <p>The failure this guards against is quiet. Reading the motor instead still produces a heading
 * that looks plausible and moves the right way -- it is just wrong by however much lash is taken
 * up, which on a swerve module means scrub and a robot that does not quite go where it is pointed.
 */
class BacklashFeedbackTest {

  /** A module whose motor and absolute encoders disagree, as a real one with backlash does. */
  private static final class LashIO implements ModuleIO {
    private final double moduleDeg;
    private final double motorDeg;
    SwerveModuleVelocity commanded;

    LashIO(double moduleDeg, double motorDeg) {
      this.moduleDeg = moduleDeg;
      this.motorDeg = motorDeg;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.turnAbsolutePosition = Rotation2d.fromDegrees(moduleDeg);
      inputs.turnPosition = inputs.turnAbsolutePosition;
      inputs.turnMotorPosition = Rotation2d.fromDegrees(motorDeg);
      inputs.turnEncoderConnected = true;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      commanded = new SwerveModuleVelocity(0.0, rotation);
    }
  }

  private static Module moduleAt(double moduleDeg, double motorDeg) {
    Module m = new Module(new LashIO(moduleDeg, motorDeg), "Test", Constants.ModuleConfig.FRONT_LEFT);
    m.updateInputs();
    return m;
  }

  @Test
  void headingComesFromTheModuleNotTheTurnMotor() {
    // 8 degrees of lash, mid-range for this robot.
    Module module = moduleAt(30.0, 38.0);

    assertEquals(
        30.0,
        module.getAngle().getDegrees(),
        1e-9,
        "getAngle() drives every heading decision and must report the module, not the motor");
    assertEquals(
        38.0,
        module.getMotorAngle().getDegrees(),
        1e-9,
        "the motor position is still reported, so the lash stays visible in the log");
  }

  @Test
  void optimizationDecidesAgainstTheModuleAngle() {
    // Chosen so the two encoders disagree about the answer, not merely about the number. optimize()
    // flips the wheel when the error exceeds 90 degrees, so with a target of -90:
    //
    //   module at  +5 deg -> 95 deg of error -> flip, command +90 and drive backwards
    //   motor  at  -5 deg -> 85 deg of error -> no flip, command -90
    //
    // 10 degrees of lash, within what this robot actually has, straddling the boundary. Reading the
    // wrong encoder here does not produce a slightly-off heading, it produces the opposite decision.
    LashIO io = new LashIO(5.0, -5.0);
    Module module = new Module(io, "Test", Constants.ModuleConfig.FRONT_LEFT);
    module.updateInputs();

    module.runSetpoint(new SwerveModuleVelocity(1.0, Rotation2d.fromDegrees(-90.0)));

    assertEquals(
        90.0,
        io.commanded.angle.getDegrees(),
        1e-6,
        "optimize() should have flipped, which it only does if it measured from the module (+5)"
            + " rather than the motor (-5); commanded "
            + io.commanded.angle.getDegrees());
  }
}
