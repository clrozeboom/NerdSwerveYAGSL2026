package frc.robot.protocol;

/**
 * CAN identifiers for the RioBridge protocol. See the protocol table in the repo root {@code
 * README.md} — these constants are the values in that table, not independently chosen here.
 *
 * <p>Device type {@code MISCELLANEOUS} (10) and manufacturer {@code TEAM_USE} (8) match {@link
 * edu.wpi.first.wpilibj.CAN#kTeamDeviceType} and {@link edu.wpi.first.wpilibj.CAN#kTeamManufacturer};
 * the single-arg {@code CAN(deviceId)} constructor already selects them, so RioBridge code
 * doesn't need to pass them explicitly. They're recorded here anyway because the Core side has to
 * reconstruct the same arbitration IDs without a {@code CAN} instance of its own.
 */
public final class CanIds {
  private CanIds() {}

  /** Team-use device number. The RioBridge is the only device on its dedicated bus (ADR-0002). */
  public static final int DEVICE_NUMBER = 1;

  /**
   * {@code apiId} arguments for {@link edu.wpi.first.wpilibj.CAN#writePacket(byte[], int)}. Each
   * is a 10-bit value; {@code CAN} shifts it left 6 bits and OR's in the device number to form the
   * 29-bit arbitration ID.
   */
  public static final int STATUS_API_ID = 0x00;

  public static final int ENCODERS_API_ID = 0x10;
  public static final int ATTITUDE_API_ID = 0x11;

  /** Resulting arbitration IDs, for reference and for the Core-side stream session. */
  public static final int STATUS_ARBITRATION_ID = 0x0A080001;

  public static final int ENCODERS_ARBITRATION_ID = 0x0A080401;
  public static final int ATTITUDE_ARBITRATION_ID = 0x0A080441;

  /**
   * Mask for the Core's single {@code openCANStreamSession}: keeps device type, manufacturer and
   * device number while ignoring the API bits, so one session catches all three RioBridge frames
   * and filters out the RIO heartbeat (device type {@code ROBOT_CONTROLLER} = 1) for free.
   */
  public static final int STREAM_MASK = 0x1FFF003F;

  public static final int STATUS_HZ = 20;
  public static final int ENCODERS_HZ = 100;
  public static final int ATTITUDE_HZ = 100;
}
