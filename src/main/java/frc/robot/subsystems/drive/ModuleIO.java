// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;

/**
 * Hardware abstraction for one swerve module.
 *
 * <p>This follows the AdvantageKit IO pattern: everything the module reads from hardware lands in a
 * plain {@link ModuleIOInputs} struct once per loop, and everything it writes goes through a small
 * set of setters. {@link Module} contains all the logic and talks only to this interface, so the
 * same control code runs against real SPARK MAXes ({@link ModuleIOSpark}) and against the physics
 * sim ({@link ModuleIOSim}) without changing.
 *
 * <p>{@code ModuleIOInputs} is deliberately a flat struct of primitives and value types. If this
 * project later takes on the AdvantageKit dependency, annotating it {@code @AutoLog} is all that is
 * needed to get replay logging; nothing else here has to change.
 */
public interface ModuleIO {

  /** Everything read from one module's hardware in a single loop. */
  class ModuleIOInputs {
    /** True when the drive controller answered this cycle. */
    public boolean driveConnected = false;

    /** Wheel travel, in radians of wheel rotation. */
    public double drivePositionRad = 0.0;

    /** Wheel speed, in radians per second of wheel rotation. */
    public double driveVelocityRadPerSec = 0.0;

    /** Voltage the drive motor is currently applying. */
    public double driveAppliedVolts = 0.0;

    /** Drive motor supply current, in amps. */
    public double driveCurrentAmps = 0.0;

    /** True when the turn controller answered this cycle. */
    public boolean turnConnected = false;

    /** True when the absolute encoder is returning a usable reading. */
    public boolean turnEncoderConnected = false;

    /** Absolute module heading, already offset-corrected. */
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;

    /** Module heading as tracked by the turn motor's own encoder. */
    public Rotation2d turnPosition = Rotation2d.kZero;

    /** Turn speed, in radians per second of module rotation. */
    public double turnVelocityRadPerSec = 0.0;

    /** Voltage the turn motor is currently applying. */
    public double turnAppliedVolts = 0.0;

    /** Turn motor supply current, in amps. */
    public double turnCurrentAmps = 0.0;
  }

  /** Refreshes {@code inputs} from hardware. Called exactly once per loop, before anything else. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Drives the wheel open-loop at the given voltage. Used by characterization routines. */
  default void setDriveOpenLoop(double volts) {}

  /** Steers the module open-loop at the given voltage. */
  default void setTurnOpenLoop(double volts) {}

  /** Commands a closed-loop wheel speed, in radians per second of wheel rotation. */
  default void setDriveVelocity(double velocityRadPerSec) {}

  /** Commands a closed-loop module heading. */
  default void setTurnPosition(Rotation2d rotation) {}

  /** Switches the drive motor between brake and coast. */
  default void setBrakeMode(boolean enabled) {}
}
