package frc.robot.subsystems.drive.riobridge.diagnostics;

import frc.robot.protocol.CanIds;
import org.wpilib.hardware.hal.can.CANJNI;
import org.wpilib.hardware.hal.can.CANStreamMessage;
import org.wpilib.hardware.hal.can.CANStreamOverflowException;
import org.wpilib.system.Timer;

/**
 * Root RioBridge README "to verify" item 1: whether {@code CANStreamMessage.timestamp} is really
 * milliseconds (per the field's own javadoc) or nanoseconds (per {@code setStreamData}'s
 * parameter javadoc, on the same class) -- confirmed on real hardware to be neither: microseconds
 * (see the pass criteria below). This class is the one thing in this package still using the
 * buffered stream session API ({@code CANStreamMessage}/{@code readCANStreamSession}) -- {@link
 * frc.robot.subsystems.drive.riobridge.RioBridgeCan} moved to the per-device {@code CAN}/{@code
 * CANReceiveMessage} API instead, after the stream session was found to never deliver payload
 * bytes on this WPILib build (see its class javadoc). That finding doesn't touch this class:
 * this only ever reads {@code .timestamp}, never {@code .data}/{@code .length}, which is exactly
 * why it kept working when {@code RioBridgeCan} didn't.
 *
 * <p>Opens its own short-lived stream session filtered to just the Status frame (20 Hz, so a
 * clean ~50 ms period) and compares elapsed raw timestamp against elapsed wall-clock time over
 * several dozen frames. Doesn't touch {@link frc.robot.subsystems.drive.riobridge.RioBridgeCan}
 * at all, so it's safe to run before or independent of wiring that up.
 *
 * <p>Run this with the RioBridge actually powered and transmitting on the bus you pass in.
 * {@link DiagnosticsRobot} is a ready driver for it.
 */
public final class TimestampUnitsCheck {
  private TimestampUnitsCheck() {}

  /** ~2 seconds of Status frames at its 20 Hz rate (CanIds.STATUS_HZ). */
  private static final int SAMPLE_COUNT = 40;

  private static final double POLL_INTERVAL_SECONDS = 0.010;

  public record Result(
      long rawTimestampDelta, double wallClockDeltaSeconds, double secondsPerUnit, String verdict) {
    public boolean succeeded() {
      return !verdict.startsWith("FAILED");
    }
  }

  /**
   * @param bus a raw HAL bus id (e.g. {@code org.wpilib.hardware.hal.CANBusMap.CAN_S1}) -- see
   *     {@code RioBridgeCan}'s javadoc for why this takes an {@code int} rather than a {@code
   *     CANPort} on this project's alpha-6 WPILib pin.
   * @param timeoutSeconds give up and return a FAILED result after this long even if fewer than
   *     {@link #SAMPLE_COUNT} Status frames arrived (most likely: RioBridge isn't powered, isn't
   *     on this bus, or isn't transmitting).
   */
  public static Result run(int bus, double timeoutSeconds) {
    int sessionHandle =
        CANJNI.openCANStreamSession(
            bus, CanIds.STATUS_ARBITRATION_ID, /* exact match, no API bits */ 0x1FFFFFFF, 8);
    try {
      return collect(sessionHandle, timeoutSeconds);
    } finally {
      CANJNI.closeCANStreamSession(sessionHandle);
    }
  }

  private static Result collect(int sessionHandle, double timeoutSeconds) {
    CANStreamMessage[] scratch = new CANStreamMessage[8];
    for (int i = 0; i < scratch.length; i++) {
      scratch[i] = new CANStreamMessage();
    }

    long firstRawTimestamp = 0;
    long lastRawTimestamp = 0;
    double firstWallClock = 0;
    double lastWallClock = 0;
    int received = 0;
    double deadline = Timer.getMonotonicTimestamp() + timeoutSeconds;

    while (received < SAMPLE_COUNT && Timer.getMonotonicTimestamp() < deadline) {
      CANStreamMessage[] batch = scratch;
      int count;
      try {
        count = CANJNI.readCANStreamSession(sessionHandle, scratch, scratch.length);
      } catch (CANStreamOverflowException overflow) {
        batch = overflow.getMessages();
        count = overflow.getMessagesRead();
      }
      for (int i = 0; i < count; i++) {
        double now = Timer.getMonotonicTimestamp();
        if (received == 0) {
          firstRawTimestamp = batch[i].timestamp;
          firstWallClock = now;
        }
        lastRawTimestamp = batch[i].timestamp;
        lastWallClock = now;
        received++;
      }
      // The session buffers between polls (that's the point of a stream session), so sleeping
      // here doesn't cost us frames the way sleeping in a tight sensor loop would -- it just
      // saves us from busy-spinning a core for the ~2 seconds this takes.
      if (received < SAMPLE_COUNT) {
        sleepQuietly();
      }
    }

    if (received < 2) {
      return new Result(
          0,
          0,
          0,
          "FAILED: received "
              + received
              + " Status frames in "
              + timeoutSeconds
              + "s -- is the RioBridge powered, wired to this bus, and running?");
    }

    long rawDelta = lastRawTimestamp - firstRawTimestamp;
    double wallClockDelta = lastWallClock - firstWallClock;
    double secondsPerUnit = wallClockDelta / rawDelta;
    String verdict = classify(secondsPerUnit, received, rawDelta, wallClockDelta);
    return new Result(rawDelta, wallClockDelta, secondsPerUnit, verdict);
  }

  /**
   * Pure classification logic, pulled out so it's unit testable without a CAN session: given the
   * ratio of wall-clock seconds elapsed to raw timestamp units elapsed, which unit does that
   * ratio actually match? Package-visible for tests.
   */
  static String classify(double secondsPerUnit, int sampleCount, long rawDelta, double wallClockDelta) {
    if (closeTo(secondsPerUnit, 1e-3, 0.3)) {
      return "MILLISECONDS (secondsPerUnit ~= 1e-3): matches the field javadoc.";
    } else if (closeTo(secondsPerUnit, 1e-9, 0.3)) {
      return "NANOSECONDS (secondsPerUnit ~= 1e-9): matches setStreamData's parameter javadoc.";
    } else if (closeTo(secondsPerUnit, 1e-6, 0.3)) {
      return "MICROSECONDS (secondsPerUnit ~= 1e-6): neither javadoc claimed this -- worth a "
          + "second look before trusting it.";
    } else {
      return "UNRECOGNIZED unit -- secondsPerUnit = "
          + secondsPerUnit
          + " (not ms, us, or ns). Recheck the raw values by hand: over "
          + sampleCount
          + " frames, raw timestamp moved by "
          + rawDelta
          + " while "
          + wallClockDelta
          + "s of wall-clock time passed.";
    }
  }

  private static boolean closeTo(double value, double expected, double relativeTolerance) {
    return Math.abs(value - expected) <= expected * relativeTolerance;
  }

  private static void sleepQuietly() {
    try {
      Thread.sleep((long) (POLL_INTERVAL_SECONDS * 1000));
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }
}
