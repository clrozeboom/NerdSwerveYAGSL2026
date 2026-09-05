package frc.robot.subsystems.drive.riobridge;

import frc.robot.protocol.CanFrames.EncodersFrame;
import frc.robot.protocol.CanFrames.StatusFrame;
import frc.robot.protocol.CanIds;
import java.util.List;
import org.wpilib.hardware.hal.can.CANJNI;
import org.wpilib.hardware.hal.can.CANStreamMessage;
import org.wpilib.hardware.hal.can.CANStreamOverflowException;

/**
 * Owns the RioBridge's single CAN stream session (root README: "The Core reads every frame with
 * a single buffered, timestamped stream session"). Construct one per robot and share it with
 * whatever reads the encoders (see {@link #latestEncoders()}) as well as {@link
 * GyroIORioBridge} -- don't open a second session.
 *
 * <p>Takes a raw HAL bus id ({@code int}) rather than {@code org.wpilib.hardware.bus.CANPort}:
 * this project is pinned to WPILib {@code 2027.0.0-alpha-6}, and that friendly enum wrapper
 * doesn't exist yet at alpha-6 -- only added to {@code wpilibj-java} between alpha-6 and alpha-7
 * (confirmed by decompiling both jars' {@code org.wpilib.hardware.bus} packages). The underlying
 * capability this class actually needs is present at alpha-6, though: {@code CANJNI}'s native
 * methods (`openCANStreamSession` included) already take a raw bus id `int` as their first
 * argument at alpha-6, and {@code org.wpilib.hardware.hal.CANBusMap} (bundled in {@code hal-java},
 * already on this project's classpath) exposes the same {@code CAN_S0}/{@code CAN_S1}/... values
 * {@code CANPort} would, just as plain {@code int} constants instead of enum entries. Pass one of
 * those (e.g. {@code CANBusMap.CAN_S1}) as {@code bus} below. If this project bumps to alpha-7 or
 * later, this constructor can switch to taking {@code CANPort} directly and drop this note.
 *
 * <p>The actual demux/decode logic lives in {@link RioBridgeCanDemux}, which has no JNI in it and
 * is unit tested directly. This class is the thin part that couldn't be build-verified against a
 * real CAN bus in the RioBridge sandbox this was ported from -- see that repo's
 * core-integration/README.md.
 */
public class RioBridgeCan implements AutoCloseable {
  private final int sessionHandle;
  private final CANStreamMessage[] scratch;
  private final RioBridgeCanDemux demux = new RioBridgeCanDemux();
  private int overflowCount = 0;

  /**
   * @param bus a raw HAL bus id, e.g. {@code org.wpilib.hardware.hal.CANBusMap.CAN_S1} -- not a
   *     {@code CANPort}, which doesn't exist at this project's alpha-6 WPILib pin. See the class
   *     javadoc.
   */
  public RioBridgeCan(int bus, int maxMessagesPerPoll) {
    sessionHandle =
        CANJNI.openCANStreamSession(
            bus, CanIds.STATUS_ARBITRATION_ID, CanIds.STREAM_MASK, maxMessagesPerPoll);
    scratch = new CANStreamMessage[maxMessagesPerPoll];
    for (int i = 0; i < scratch.length; i++) {
      scratch[i] = new CANStreamMessage();
    }
  }

  @Override
  public void close() {
    CANJNI.closeCANStreamSession(sessionHandle);
  }

  /** Reads and demultiplexes every frame the session has buffered since the last call. */
  public void poll() {
    CANStreamMessage[] received = scratch;
    int messagesRead;
    try {
      messagesRead = CANJNI.readCANStreamSession(sessionHandle, scratch, scratch.length);
    } catch (CANStreamOverflowException overflow) {
      // The session's buffer filled between polls, so some frames were dropped -- exactly the
      // dropped-frame case the RioBridge protocol already designs for (it just shows up as a
      // bigger-than-usual gap between two attitude timestamps). Demux what we did get rather than
      // discarding it.
      received = overflow.getMessages();
      messagesRead = overflow.getMessagesRead();
      overflowCount++;
    }
    for (int i = 0; i < messagesRead; i++) {
      demux.accept(received[i]);
    }
  }

  /**
   * How many times {@link #poll()} has hit {@link CANStreamOverflowException} -- i.e. this
   * session's buffer filled between two polls and at least one frame was dropped before being
   * read. Should stay at 0 in normal operation; a nonzero, growing count means the shared SPI
   * master isn't keeping up with this bus.
   */
  public int overflowCount() {
    return overflowCount;
  }

  public StatusFrame latestStatus() {
    return demux.latestStatus();
  }

  public double latestStatusTimestampSeconds() {
    return demux.latestStatusTimestampSeconds();
  }

  public EncodersFrame latestEncoders() {
    return demux.latestEncoders();
  }

  public double latestEncodersTimestampSeconds() {
    return demux.latestEncodersTimestampSeconds();
  }

  public AttitudeSample latestAttitude() {
    return demux.latestAttitude();
  }

  /**
   * Every Attitude sample received since the last call, oldest first, for AdvantageKit's
   * per-sample odometry arrays. Call once per {@code updateInputs} -- this drains the buffer.
   */
  public List<AttitudeSample> drainAttitudeSamples() {
    return demux.drainAttitudeSamples();
  }
}
