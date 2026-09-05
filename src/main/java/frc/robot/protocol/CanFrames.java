package frc.robot.protocol;

import java.util.Arrays;

/**
 * Wire format for the three RioBridge CAN frames, matching the protocol table in the repo root
 * {@code README.md}. All multi-byte fields are little-endian.
 *
 * <p>This is plain byte-packing logic with no WPILib dependency, on purpose: it is unit tested
 * directly here, and the Core side needs the identical encode/decode logic despite building
 * against different WPILib jars. Keep {@code core-integration/protocol/CanFrames.java}
 * byte-for-byte identical to this file — see the note at the top of that copy.
 */
public final class CanFrames {
  private CanFrames() {}

  public static final int PROTOCOL_VERSION = 1;

  /**
   * Status frame flag bits (bytes 4-5). Which sensors the RioBridge believes are present is
   * "ours to design" per {@code CONTEXT.md}; only navX liveness is populated for now; the
   * remaining bits are reserved and must be sent as 0 until a sensor gains real presence
   * detection.
   */
  public static final int FLAG_NAVX_CONNECTED = 0x0001;

  private static final int FRAME_LENGTH = 8;

  // ---- Status: loopCounter u16 | uptime u16 | flags u16 | protocolVersion u16 ----

  public static byte[] packStatus(int loopCounter, int uptimeSeconds, int flags, int protocolVersion) {
    byte[] out = new byte[FRAME_LENGTH];
    putU16LE(out, 0, loopCounter);
    putU16LE(out, 2, uptimeSeconds);
    putU16LE(out, 4, flags);
    putU16LE(out, 6, protocolVersion);
    return out;
  }

  public record StatusFrame(int loopCounter, int uptimeSeconds, int flags, int protocolVersion) {
    public boolean navxConnected() {
      return (flags & FLAG_NAVX_CONNECTED) != 0;
    }
  }

  public static StatusFrame unpackStatus(byte[] data) {
    requireLength(data);
    return new StatusFrame(
        getU16LE(data, 0), getU16LE(data, 2), getU16LE(data, 4), getU16LE(data, 6));
  }

  // ---- Encoders: analog[0..3] u16, raw 12-bit ADC counts ----

  /**
   * Packs four raw ADC counts (0-4095 on roboRIO analog input hardware). Values outside u16
   * range are truncated to their low 16 bits rather than rejected, matching {@link
   * #packStatus} — RioBridge must never throw out of its main loop over a sensor reading.
   */
  public static byte[] packEncoders(int[] rawCounts) {
    if (rawCounts.length != 4) {
      throw new IllegalArgumentException("expected 4 encoder channels, got " + rawCounts.length);
    }
    byte[] out = new byte[FRAME_LENGTH];
    for (int i = 0; i < 4; i++) {
      putU16LE(out, i * 2, rawCounts[i]);
    }
    return out;
  }

  public record EncodersFrame(int[] rawCounts) {
    public EncodersFrame {
      if (rawCounts.length != 4) {
        throw new IllegalArgumentException("expected 4 encoder channels, got " + rawCounts.length);
      }
      rawCounts = Arrays.copyOf(rawCounts, 4);
    }
  }

  public static EncodersFrame unpackEncoders(byte[] data) {
    requireLength(data);
    int[] raw = new int[4];
    for (int i = 0; i < 4; i++) {
      raw[i] = getU16LE(data, i * 2);
    }
    return new EncodersFrame(raw);
  }

  // ---- Attitude: yaw/yawRate/pitch/roll, each i16 ----

  private static final double YAW_LSB_DEG = 0.01;
  private static final double YAW_RATE_LSB_DEG_PER_SEC = 0.1;
  private static final double PITCH_ROLL_LSB_DEG = 0.01;

  public static byte[] packAttitude(
      double yawDeg, double yawRateDegPerSec, double pitchDeg, double rollDeg) {
    byte[] out = new byte[FRAME_LENGTH];
    putI16LE(out, 0, scaleClamped(yawDeg, YAW_LSB_DEG));
    putI16LE(out, 2, scaleClamped(yawRateDegPerSec, YAW_RATE_LSB_DEG_PER_SEC));
    putI16LE(out, 4, scaleClamped(pitchDeg, PITCH_ROLL_LSB_DEG));
    putI16LE(out, 6, scaleClamped(rollDeg, PITCH_ROLL_LSB_DEG));
    return out;
  }

  public record AttitudeFrame(double yawDeg, double yawRateDegPerSec, double pitchDeg, double rollDeg) {}

  public static AttitudeFrame unpackAttitude(byte[] data) {
    requireLength(data);
    return new AttitudeFrame(
        getI16LE(data, 0) * YAW_LSB_DEG,
        getI16LE(data, 2) * YAW_RATE_LSB_DEG_PER_SEC,
        getI16LE(data, 4) * PITCH_ROLL_LSB_DEG,
        getI16LE(data, 6) * PITCH_ROLL_LSB_DEG);
  }

  // ---- little-endian helpers ----

  private static void putU16LE(byte[] buf, int offset, int value) {
    buf[offset] = (byte) (value & 0xFF);
    buf[offset + 1] = (byte) ((value >>> 8) & 0xFF);
  }

  private static int getU16LE(byte[] buf, int offset) {
    return (buf[offset] & 0xFF) | ((buf[offset + 1] & 0xFF) << 8);
  }

  private static void putI16LE(byte[] buf, int offset, int value) {
    putU16LE(buf, offset, value & 0xFFFF);
  }

  private static int getI16LE(byte[] buf, int offset) {
    return (short) getU16LE(buf, offset);
  }

  /** Rounds to the nearest LSB and clamps to the i16 range rather than overflowing silently. */
  private static int scaleClamped(double valueDeg, double lsb) {
    double scaled = Math.round(valueDeg / lsb);
    return (int) Math.max(Short.MIN_VALUE, Math.min(Short.MAX_VALUE, scaled));
  }

  private static void requireLength(byte[] data) {
    if (data.length != FRAME_LENGTH) {
      throw new IllegalArgumentException("expected " + FRAME_LENGTH + " bytes, got " + data.length);
    }
  }
}
