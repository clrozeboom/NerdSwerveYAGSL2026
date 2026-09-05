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
 * <p><b>An actual buffer overflow crashes the JVM on real hardware -- confirmed, not
 * theoretical.</b> {@link #poll}'s {@code catch (CANStreamOverflowException)} below was written
 * assuming a dropped frame is a survivable, tolerated case, the same way a missed UDP packet
 * would be. That's wrong for this specific exception on this WPILib build: a real overflow (an
 * earlier version of {@code DiagnosticsRobot} left its own session unpolled for ~2.8s while
 * transmitting at ~220 frames/sec, far past its old 32-message buffer) segfaulted before the
 * catch block ever ran -- {@code SIGSEGV}, {@code SEGV_MAPERR}, address {@code 0x0}, inside
 * {@code wpi::hal::ThrowCANStreamOverflowException}'s call to {@code JNIEnv_::NewObject} in
 * {@code libwpiHaljni.so}, called from {@code CANJNI.readCANStreamSession}. The native code
 * that's supposed to construct and throw this exception null-derefs instead, so nothing in Java
 * -- no try/catch here or anywhere else -- ever gets a chance to run. That makes avoiding a real
 * overflow a hard requirement, not a nice-to-have: size {@code maxMessagesPerPoll} generously
 * (comfortably above frames-per-second times the longest realistic gap between polls, not just
 * the steady-state case) and never construct a session long before you start polling it, the way
 * {@code DiagnosticsRobot} used to. {@link #overflowCount()} should read as "this never actually
 * happened," not "this happened and got handled."
 *
 * <p>The actual demux/decode logic lives in {@link RioBridgeCanDemux}, which has no JNI in it and
 * is unit tested directly. This class is the thin part that couldn't be build-verified against a
 * real CAN bus in the RioBridge sandbox this was ported from -- see that repo's
 * core-integration/README.md, though the overflow crash above confirms it does run against a
 * real one now.
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
      // This catch block is a courtesy, not a safety net -- see the class javadoc. On real
      // hardware, actually reaching this condition has crashed the whole JVM before this code
      // ever ran, at the native layer that's supposed to construct the exception in the first
      // place. If you're seeing this catch actually execute, either that native bug got fixed
      // upstream, or you got lucky on timing -- don't take it as confirmation this is safe to
      // rely on. Demux what we did get rather than discarding it.
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
   * read. Must stay at 0: this isn't "a nonzero count means dropped data," it's "a nonzero count
   * means you got lucky that this didn't crash the JVM instead" -- see the class javadoc. A
   * nonzero, growing count is still the direct answer to whether the shared SPI master is
   * keeping up, but the goal on real hardware is never seeing it move off 0 in the first place.
   */
  public int overflowCount() {
    return overflowCount;
  }

  /**
   * How many messages {@link #poll()} has received that matched one of the RioBridge protocol's
   * three arbitration IDs but weren't actually shaped like that frame -- see {@link
   * RioBridgeCanDemux}'s class javadoc for the confirmed-on-real-hardware case this guards
   * against. Should stay at 0.
   */
  public int malformedFrameCount() {
    return demux.malformedFrameCount();
  }

  /** See {@link RioBridgeCanDemux#lastMalformedFrameDescription()}. */
  public String lastMalformedFrameDescription() {
    return demux.lastMalformedFrameDescription();
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
