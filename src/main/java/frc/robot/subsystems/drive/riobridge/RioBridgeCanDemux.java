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
 *
 * <p><b>A message can match one of the three arbitration IDs and still not be shaped like that
 * frame -- confirmed on real hardware, and confirmed systemic, not a rare edge case.</b> A live
 * run hit this on essentially every frame (a malformed count climbing by ~220/sec, matching the
 * protocol's entire combined rate, with zero Attitude samples getting through) -- not the
 * occasional bad frame the first sighting looked like. {@link #accept} drops a frame that fails
 * to unpack instead of letting the {@code IllegalArgumentException} escape and kill the whole
 * robot program (the same "don't throw out of a periodic loop" call already made below for an
 * unrecognized arbitration ID entirely), but that's damage control, not a fix -- at this rate,
 * essentially no real RioBridge data is reaching the Core. Leading hypothesis, not yet confirmed:
 * {@code TimestampUnitsCheck} -- the one piece of this whole investigation that has actually
 * worked against a real stream session -- only ever reads {@link CANStreamMessage#timestamp}; it
 * never touches {@code .data}/{@code .length}. {@link RioBridgeCan}'s session is the only code
 * path here that has ever read those two fields from a real session, and it has never once
 * succeeded. That's consistent with this WPILib build's {@code readCANStreamSession} populating
 * timestamp/messageId correctly but not marshaling the payload bytes back to Java at all -- which
 * would make this a structural blocker for the whole design (which depends on reading an 8-byte
 * payload per frame), not a per-frame nuisance. {@link #lastMalformedFrameDescription()} exists
 * to confirm or rule this out on the next run: if {@code dataLength} is consistently 0 across
 * different arbitration IDs and timestamps look sane and increasing, that's the marshaling gap;
 * if lengths vary or timestamps look wrong too, something else is going on. This is the third
 * real, hardware-found rough edge in this exact {@code readCANStreamSession} code path, after the
 * timestamp-unit ambiguity above and {@link RioBridgeCan}'s overflow-segfault.
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
  private int malformedFrameCount = 0;
  private String lastMalformedFrameDescription;

  void accept(CANStreamMessage message) {
    byte[] data = Arrays.copyOf(message.data, message.length);
    double timestampSeconds = message.timestamp * TIMESTAMP_TO_SECONDS;
    int id = message.messageId & ARBITRATION_ID_MASK;

    try {
      if (id == CanIds.STATUS_ARBITRATION_ID) {
        latestStatus = CanFrames.unpackStatus(data);
        latestStatusTimestampSeconds = timestampSeconds;
      } else if (id == CanIds.ENCODERS_ARBITRATION_ID) {
        latestEncoders = CanFrames.unpackEncoders(data);
        latestEncodersTimestampSeconds = timestampSeconds;
      } else if (id == CanIds.ATTITUDE_ARBITRATION_ID) {
        AttitudeSample sample =
            new AttitudeSample(CanFrames.unpackAttitude(data), timestampSeconds);
        latestAttitude = sample;
        pendingAttitudeSamples.add(sample);
      }
      // Anything else matched the session's coarse filter without being one of our three frames
      // -- shouldn't happen given the mask in CanIds, but silently ignoring it is the right
      // failure mode on an offseason bridge, not throwing out of a periodic loop.
    } catch (IllegalArgumentException malformed) {
      // Matched one of our three arbitration IDs but wasn't actually shaped like that frame (see
      // class javadoc). Drop this one frame and keep the previous latest* value rather than
      // crash the whole loop over one bad frame; malformedFrameCount/lastMalformedFrameDescription
      // are the visibility into how often this happens and what the raw message actually looked
      // like when it did.
      malformedFrameCount++;
      lastMalformedFrameDescription =
          String.format(
              "arbitrationId=0x%X rawMessageId=0x%X dataLength=%d rawTimestamp=%d: %s",
              id, message.messageId, message.length, message.timestamp, malformed.getMessage());
    }
  }

  /**
   * How many times {@link #accept} has received a message that matched one of the RioBridge
   * protocol's three arbitration IDs but failed to unpack as that frame's expected byte length.
   * Should stay at 0 -- confirmed on real hardware to instead climb at essentially the protocol's
   * full frame rate (see class javadoc), not the rare one-off this originally looked like. Not a
   * RioBridge sender-side bug -- {@code CanFrames}' packing methods always emit exactly 8 bytes.
   */
  int malformedFrameCount() {
    return malformedFrameCount;
  }

  /**
   * Details of the most recent frame {@link #accept} couldn't unpack, or {@code null} if none
   * has happened yet. Print this (throttled -- it changes on every malformed frame when they're
   * frequent) to test the class javadoc's marshaling-gap hypothesis: consistently
   * {@code dataLength=0} with sane, increasing {@code rawTimestamp} values across different
   * {@code arbitrationId}s would confirm it; varying lengths or nonsensical timestamps would
   * point elsewhere instead.
   */
  String lastMalformedFrameDescription() {
    return lastMalformedFrameDescription;
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
