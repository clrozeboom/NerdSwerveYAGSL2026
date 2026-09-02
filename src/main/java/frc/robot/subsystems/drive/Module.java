// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * One swerve module's logic, sitting on top of a {@link ModuleIO}.
 *
 * <p>This class holds everything that is the same whether the module is real or simulated: unit
 * conversion between wheel radians and metres, angle optimization, and publishing telemetry. It
 * never touches a motor controller directly.
 */
public class Module {
  // The PID gains are shared. They describe how hard the controller pushes on an error, not a
  // property of the hardware, so four different values would be four things to keep in step for no
  // physical reason — and one set of dashboard entries is much easier to drive while tuning.
  private static final TunableNumber driveKp =
      new TunableNumber("Tuning/Drive/kP", Constants.Module.DRIVE_KP);
  private static final TunableNumber driveKd =
      new TunableNumber("Tuning/Drive/kD", Constants.Module.DRIVE_KD);
  private static final TunableNumber turnKp =
      new TunableNumber("Tuning/Turn/kP", Constants.Module.TURN_KP);
  private static final TunableNumber turnKd =
      new TunableNumber("Tuning/Turn/kD", Constants.Module.TURN_KD);

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;
  private final Constants.ModuleConfig config;

  // The feedforward gains are per module, because they describe this corner's hardware. Static
  // friction in particular varies enough between modules to be worth carrying separately; see
  // ModuleConfig.driveKs.
  private final TunableNumber driveKs;
  private final TunableNumber driveKv;

  public Module(ModuleIO io, String name, Constants.ModuleConfig config) {
    this.io = io;
    this.name = name;
    this.config = config;
    this.driveKs = new TunableNumber("Tuning/Drive/" + name + "/kS", config.driveKs);
    this.driveKv = new TunableNumber("Tuning/Drive/" + name + "/kV", config.driveKv);

    // Both IO implementations already apply the compiled-in gains when they are constructed, so
    // consume the initial "changed" state here. Without this the first periodic() would re-push all
    // six gains, which on real hardware is eight blocking CAN configure() calls landing in the first
    // loop of the match.
    TunableNumber.anyChanged(hashCode(), driveKp, driveKd, driveKs, driveKv);
    TunableNumber.anyChanged(hashCode(), turnKp, turnKd);
  }

  /**
   * Reads hardware and publishes this module's inputs. Called once per loop from {@link Drive}
   * before any control runs, so every calculation in a given loop sees the same snapshot.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
    // Logger.processInputs writes these values to the log when running, and replaces them with the
    // logged values when replaying. That single call is what makes replay work.
    Logger.processInputs("Drive/" + name, inputs);

    Logger.recordOutput("Drive/" + name + "/DrivePositionMeters", getPositionMeters());
    Logger.recordOutput("Drive/" + name + "/DriveVelocityMetersPerSec", getVelocityMetersPerSec());
    Logger.recordOutput("Drive/" + name + "/TurnPositionDeg", getAngle().getDegrees());

    // Push edited gains down to the controller, but only when something actually changed — on a
    // SPARK MAX each push is a CAN transaction.
    if (TunableNumber.anyChanged(hashCode(), driveKp, driveKd, driveKs, driveKv)) {
      io.setDriveGains(driveKp.get(), driveKd.get(), driveKs.get(), driveKv.get());
    }
    if (TunableNumber.anyChanged(hashCode(), turnKp, turnKd)) {
      io.setTurnGains(turnKp.get(), turnKd.get());
    }
  }

  /**
   * Commands a module state, taking the shorter path to the requested angle.
   *
   * @param state the desired wheel speed and module heading
   */
  public void runSetpoint(SwerveModuleVelocity state) {
    SwerveModuleVelocity optimized = state.optimize(getAngle());
    // Scale the wheel speed down while the module is still turning into place, so the robot does not
    // lurch sideways during the first few milliseconds of a direction change.
    optimized = optimized.cosineScale(getAngle());

    io.setDriveVelocity(optimized.velocity / Constants.Module.WHEEL_RADIUS);
    io.setTurnPosition(optimized.angle);
  }

  /**
   * Runs the drive motor open-loop while holding the module straight ahead. Used by the
   * characterization routine.
   *
   * @param volts voltage to apply to the drive motor
   */
  public void runCharacterization(double volts) {
    io.setDriveOpenLoop(volts);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /**
   * Applies an open-loop drive voltage without touching the turn controller, so the caller can hold
   * the module at an angle of its choosing. Used by the spin characterization.
   *
   * @param volts voltage to apply to the drive motor
   */
  public void runCharacterizationDriveOnly(double volts) {
    io.setDriveOpenLoop(volts);
  }

  /** Stops both motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Switches the drive motor between brake and coast. */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Declares this module to be pointing straight forward right now. See {@link ModuleIO#zeroTurnEncoder()}. */
  public void zeroTurnEncoder() {
    io.zeroTurnEncoder();
  }

  /** Current module heading. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Distance the wheel has travelled, in metres. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * Constants.Module.WHEEL_RADIUS;
  }

  /** Current wheel speed, in metres per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * Constants.Module.WHEEL_RADIUS;
  }

  /** Wheel travel and heading, for odometry. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Wheel speed and heading, for chassis-velocity estimation. */
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Wheel travel in radians, used by the characterization routine. */
  public double getWheelRadiansForCharacterization() {
    return inputs.drivePositionRad;
  }

  /**
   * Absolute encoder heading with the configured offset already applied. Reads zero when the module
   * points forward and the offset is correct, which is what the offset calibration routine checks.
   */
  public Rotation2d getAbsolutePosition() {
    return inputs.turnAbsolutePosition;
  }

  /** This module's name, for logging and for the calibration report. */
  public String getName() {
    return name;
  }

  /** This module's hardware configuration: CAN ids, encoder channel, offset and feedforward gains. */
  public Constants.ModuleConfig getConfig() {
    return config;
  }

  /** This corner's live static-friction gain, so the ramp can report what it was measured against. */
  public double getDriveKs() {
    return driveKs.get();
  }

  /** This corner's live velocity gain. */
  public double getDriveKv() {
    return driveKv.get();
  }

  /** Commands a raw module heading, bypassing optimization. Used by the turn step-response test. */
  public void runTurnSetpoint(Rotation2d angle) {
    io.setTurnPosition(angle);
  }

  /** Commands a raw wheel speed in rad/s, bypassing kinematics. Used by the drive step test. */
  public void runDriveSetpoint(double velocityRadPerSec) {
    io.setDriveVelocity(velocityRadPerSec);
  }

  /** True when both controllers answered on the last cycle. */
  public boolean isConnected() {
    return inputs.driveConnected && inputs.turnConnected;
  }
}
