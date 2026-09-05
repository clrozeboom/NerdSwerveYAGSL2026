package frc.robot.subsystems.drive.riobridge;

import frc.robot.protocol.CanFrames.AttitudeFrame;

/** One decoded Attitude frame, paired with the Core-side timestamp it arrived with. */
public record AttitudeSample(AttitudeFrame attitude, double timestampSeconds) {}
