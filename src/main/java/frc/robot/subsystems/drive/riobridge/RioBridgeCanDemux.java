package frc.robot.subsystems.drive.riobridge;

import frc.robot.protocol.CanFrames;
import frc.robot.protocol.CanFrames.EncodersFrame;
import frc.robot.protocol.CanFrames.StatusFrame;
import frc.robot.protocol.CanIds;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.wpilib.hardware.hal.can.CANStreamMessage;

/**
 * The RioBridge protocol's demux/decode state: turns received {@link CANStreamMessage}s into the
 * three frame types. Deliberately has no JNI calls in it -- {@link RioBridgeCan} is the thin
 * wrapper that owns the actual CAN stream session and feeds messages in here -- so this class is
 * exercised directly in {@code RioBridgeCanDemuxTest} without a CAN session or the desktop HAL
 * sim.
 *
 * <p>{@link CANStreamMessage#timestamp}'s unit was genuinely ambiguous from the javadoc alone --
 * the field comment says milliseconds since {@code CLOCK_MONOTONIC}, {@code setStreamData}'s
 * parameter javadoc on the same class says nanoseconds -- and confirmed neither: a real
 * {@code TimestampUnitsCheck} run against this exact bus measured {@code secondsPerUnit ~= 1e-6}
 * (wall-clock elapsed=1.946s against a raw timestamp delta of 1,950,175 over 40 Status frames at
 * 20 Hz), which is microseconds. {@link #TIMESTAMP_TO_SECONDS} follows that measurement, not
 * either javadoc.
 */
final class RioBridgeCanDemux {
  private static final double TIMESTAMP_TO_SECONDS = 1.0 / 1_000_000.0;

  /** Strips the CAN JNI's frame-type flag bits (see {@code CANJNI.CAN_IS_FRAME_*}) before
   *  comparing a received message ID against the 29-bit arbitration IDs in {@link CanIds}. */
  private static final int ARBITRATION_ID_MASK = 0x1FFFFFFF;

  private StatusFrame latestStatus;
  private double latestStatusTimestampSeconds = Double.NEGATIVE_INFINITY;
  private EncodersFrame latestEncoders;
  private double latestEncodersTimestampSeconds = Double.NEGATIVE_INFINITY;
  private AttitudeSample latestAttitude;
  private final List<AttitudeSample> pendingAttitudeSamples = new ArrayList<>();

  void accept(CANStreamMessage message) {
    byte[] data = Arrays.copyOf(message.data, message.length);
    double timestampSeconds = message.timestamp * TIMESTAMP_TO_SECONDS;
    int id = message.messageId & ARBITRATION_ID_MASK;

    if (id == CanIds.STATUS_ARBITRATION_ID) {
      latestStatus = CanFrames.unpackStatus(data);
      latestStatusTimestampSeconds = timestampSeconds;
    } else if (id == CanIds.ENCODERS_ARBITRATION_ID) {
      latestEncoders = CanFrames.unpackEncoders(data);
      latestEncodersTimestampSeconds = timestampSeconds;
    } else if (id == CanIds.ATTITUDE_ARBITRATION_ID) {
      AttitudeSample sample = new AttitudeSample(CanFrames.unpackAttitude(data), timestampSeconds);
      latestAttitude = sample;
      pendingAttitudeSamples.add(sample);
    }
    // Anything else matched the session's coarse filter without being one of our three frames --
    // shouldn't happen given the mask in CanIds, but silently ignoring it is the right failure
    // mode on an offseason bridge (ADR-0005), not throwing out of a periodic loop.
  }

  StatusFrame latestStatus() {
    return latestStatus;
  }

  double latestStatusTimestampSeconds() {
    return latestStatusTimestampSeconds;
  }

  EncodersFrame latestEncoders() {
    return latestEncoders;
  }

  double latestEncodersTimestampSeconds() {
    return latestEncodersTimestampSeconds;
  }

  AttitudeSample latestAttitude() {
    return latestAttitude;
  }

  /**
   * Every Attitude sample received since the last call, oldest first, for AdvantageKit's
   * per-sample odometry arrays. Call once per {@code updateInputs} -- this drains the buffer.
   */
  List<AttitudeSample> drainAttitudeSamples() {
    if (pendingAttitudeSamples.isEmpty()) {
      return Collections.emptyList();
    }
    List<AttitudeSample> drained = List.copyOf(pendingAttitudeSamples);
    pendingAttitudeSamples.clear();
    return drained;
  }
}
