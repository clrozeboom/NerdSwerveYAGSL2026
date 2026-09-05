package frc.robot.subsystems.drive.riobridge;

import frc.robot.protocol.CanFrames;
import frc.robot.protocol.CanFrames.EncodersFrame;
import frc.robot.protocol.CanFrames.StatusFrame;
import frc.robot.protocol.CanIds;
import java.util.Arrays;
import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.hal.can.CANReceiveMessage;

/**
 * Owns the RioBridge's CAN reads: one {@link CAN} device handle for the RioBridge's own device
 * identity (bus + device ID + manufacturer + device type), read per frame type via {@code
 * readPacketLatest} -- not the buffered stream session the original design used.
 *
 * <p><b>Why this exists: the stream session API never delivered payload data on this WPILib
 * build.</b> The previous version of this class used {@code CANJNI.openCANStreamSession}/{@code
 * readCANStreamSession} with {@code CANStreamMessage} (see git history, and {@code
 * RioBridgeCanDemux} before it was deleted here). That populated {@code
 * CANStreamMessage.timestamp}/{@code .messageId} correctly but {@code .length} was 0 on
 * essentially every real frame -- confirmed on real hardware at a rate matching the protocol's
 * entire combined frame rate (~220/sec), not an occasional glitch:
 * {@code malformedFrameCount} climbed in lockstep with the expected frame rate while {@code
 * attitudeFramesLastSecond} stayed at 0.
 *
 * <p>This class is the fallback: the older, non-streaming, per-device {@code CAN}/{@code
 * CANAPIJNI} API instead of the buffered stream session -- a structurally different native code
 * path (device-scoped {@code readCANPacketLatest} rather than a shared, mask-filtered stream
 * buffer). <b>Whether it actually marshals payload bytes correctly is exactly what deploying this
 * branch tests -- this is not yet confirmed to work, only more likely to.</b> If {@code
 * malformedFrameCount} still climbs here, the payload-marshaling gap isn't specific to the stream
 * session API, and the next step is a different WPILib version or an upstream fix, not more
 * workarounds in this class.
 *
 * <p><b>What this loses versus the stream session design:</b> per-sample buffering. {@code
 * readPacketLatest} only ever returns the single most recent packet for a given API ID -- there's
 * no history of every sample received between two polls, unlike a buffered stream session. That's
 * a real regression against the root RioBridge README's "single buffered, timestamped stream
 * session" design intent, and would matter for an AdvantageKit-style multi-sample odometry array
 * -- but this project's {@link GyroIORioBridge} already only ever reads the single latest
 * Attitude sample (see its class javadoc), so it costs nothing here specifically.
 *
 * <p>{@link CANReceiveMessage#timestamp}'s javadoc is unambiguous -- "Timestamp message was
 * received, in microseconds (wpi time)" -- unlike {@code CANStreamMessage}'s self-contradicting
 * one, and matches the microsecond measurement {@code TimestampUnitsCheck} already confirmed
 * against the stream API, so {@link #TIMESTAMP_TO_SECONDS} carries over unchanged.
 */
public class RioBridgeCan implements AutoCloseable {
  private static final double TIMESTAMP_TO_SECONDS = 1.0 / 1_000_000.0;

  private final CAN can;
  private final CANReceiveMessage statusMessage = new CANReceiveMessage();
  private final CANReceiveMessage encodersMessage = new CANReceiveMessage();
  private final CANReceiveMessage attitudeMessage = new CANReceiveMessage();

  private StatusFrame latestStatus;
  private double latestStatusTimestampSeconds = Double.NEGATIVE_INFINITY;
  private EncodersFrame latestEncoders;
  private double latestEncodersTimestampSeconds = Double.NEGATIVE_INFINITY;
  private AttitudeSample latestAttitude;
  private double lastAttitudeTimestampSeenSeconds = Double.NEGATIVE_INFINITY;
  private int newAttitudeSampleCount = 0;
  private int malformedFrameCount = 0;
  private String lastMalformedFrameDescription;

  /**
   * @param bus a raw HAL bus id, e.g. {@code org.wpilib.hardware.hal.CANBusMap.CAN_S1} -- not a
   *     {@code CANPort}, which doesn't exist at this project's alpha-6 WPILib pin (see the
   *     previous version of this class's javadoc for why that's fine).
   */
  public RioBridgeCan(int bus) {
    can = new CAN(bus, CanIds.DEVICE_NUMBER, CAN.TEAM_MANUFACTURER, CAN.TEAM_DEVICE_TYPE);
  }

  @Override
  public void close() {
    can.close();
  }

  /** Reads the latest cached packet for each of the RioBridge protocol's three frames. */
  public void poll() {
    pollStatus();
    pollEncoders();
    pollAttitude();
  }

  private void pollStatus() {
    if (!can.readPacketLatest(CanIds.STATUS_API_ID, statusMessage)) {
      return; // Nothing received yet on this API ID -- keep whatever was cached before.
    }
    try {
      latestStatus = CanFrames.unpackStatus(trim(statusMessage));
      latestStatusTimestampSeconds = statusMessage.timestamp * TIMESTAMP_TO_SECONDS;
    } catch (IllegalArgumentException malformed) {
      recordMalformed(CanIds.STATUS_API_ID, statusMessage, malformed);
    }
  }

  private void pollEncoders() {
    if (!can.readPacketLatest(CanIds.ENCODERS_API_ID, encodersMessage)) {
      return;
    }
    try {
      latestEncoders = CanFrames.unpackEncoders(trim(encodersMessage));
      latestEncodersTimestampSeconds = encodersMessage.timestamp * TIMESTAMP_TO_SECONDS;
    } catch (IllegalArgumentException malformed) {
      recordMalformed(CanIds.ENCODERS_API_ID, encodersMessage, malformed);
    }
  }

  private void pollAttitude() {
    if (!can.readPacketLatest(CanIds.ATTITUDE_API_ID, attitudeMessage)) {
      return;
    }
    try {
      double timestampSeconds = attitudeMessage.timestamp * TIMESTAMP_TO_SECONDS;
      latestAttitude = new AttitudeSample(CanFrames.unpackAttitude(trim(attitudeMessage)), timestampSeconds);
      // readPacketLatest returns the same cached packet on every call between real updates, so
      // only count it as a "new" sample when its timestamp actually moved -- the closest
      // equivalent to the old stream session's per-sample count without buffering every sample.
      if (timestampSeconds != lastAttitudeTimestampSeenSeconds) {
        lastAttitudeTimestampSeenSeconds = timestampSeconds;
        newAttitudeSampleCount++;
      }
    } catch (IllegalArgumentException malformed) {
      recordMalformed(CanIds.ATTITUDE_API_ID, attitudeMessage, malformed);
    }
  }

  private static byte[] trim(CANReceiveMessage message) {
    return Arrays.copyOf(message.data, message.length);
  }

  private void recordMalformed(int apiId, CANReceiveMessage message, IllegalArgumentException malformed) {
    malformedFrameCount++;
    lastMalformedFrameDescription =
        String.format(
            "apiId=0x%X dataLength=%d rawTimestamp=%d: %s",
            apiId, message.length, message.timestamp, malformed.getMessage());
  }

  /**
   * Always 0. This implementation has no buffered session -- {@code readPacketLatest} just
   * overwrites a single cached packet per API ID, so there's nothing to overflow. Kept so
   * existing callers (e.g. diagnostics printing) built against the stream-session version of this
   * class don't need an unrelated code path removed.
   */
  public int overflowCount() {
    return 0;
  }

  /**
   * How many times {@link #poll()} has received a packet that failed to unpack as its expected
   * frame -- see class javadoc for why this exists and what it would mean if it's still nonzero
   * with this implementation.
   */
  public int malformedFrameCount() {
    return malformedFrameCount;
  }

  /** Details of the most recent frame {@link #poll()} couldn't unpack, or {@code null} if none has happened yet. */
  public String lastMalformedFrameDescription() {
    return lastMalformedFrameDescription;
  }

  public StatusFrame latestStatus() {
    return latestStatus;
  }

  public double latestStatusTimestampSeconds() {
    return latestStatusTimestampSeconds;
  }

  public EncodersFrame latestEncoders() {
    return latestEncoders;
  }

  public double latestEncodersTimestampSeconds() {
    return latestEncodersTimestampSeconds;
  }

  public AttitudeSample latestAttitude() {
    return latestAttitude;
  }

  /**
   * How many distinct (by timestamp) Attitude samples {@link #poll()} has seen since the last
   * call to this method. Resets to 0 each call. The closest equivalent to the old stream
   * session's {@code drainAttitudeSamples().size()} without actually buffering every sample --
   * see class javadoc.
   */
  public int drainNewAttitudeSampleCount() {
    int count = newAttitudeSampleCount;
    newAttitudeSampleCount = 0;
    return count;
  }
}
