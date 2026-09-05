package frc.robot.subsystems.drive.riobridge.diagnostics;

import org.wpilib.hardware.hal.can.CANJNI;
import org.wpilib.hardware.hal.can.CANStatus;

/**
 * Root RioBridge README "to verify" item 3: whether the RioBridge's bus (CAN_S1) keeps up while
 * bus 0 (the drivetrain's SPARK MAXes) is loaded, given both HAT channels share the Pi's SPI
 * master.
 *
 * <p>Wraps {@code CANJNI.getCANStatus} -- exactly the counters the README names. Takes a raw HAL
 * bus id ({@code int}) plus a display name rather than {@code CANPort}, which doesn't exist at
 * this project's alpha-6 WPILib pin -- see {@code RioBridgeCan}'s javadoc.
 * {@link DiagnosticsRobot} is a ready driver for this.
 */
public final class BusHealthMonitor {
  private BusHealthMonitor() {}

  /** One {@code getCANStatus} reading, tagged with which bus it came from. */
  public record BusReading(
      String busName,
      double percentBusUtilization,
      int busOffCount,
      int txFullCount,
      int receiveErrorCount,
      int transmitErrorCount) {}

  public static BusReading sample(int bus, String busName) {
    CANStatus status = new CANStatus();
    CANJNI.getCANStatus(bus, status);
    return new BusReading(
        busName,
        status.percentBusUtilization,
        status.busOffCount,
        status.txFullCount,
        status.receiveErrorCount,
        status.transmitErrorCount);
  }

  /**
   * True if {@code current}, compared to {@code previous} on the same bus, shows anything that
   * indicates lost headroom since the last sample: a new bus-off event, a new TX-full event, or
   * new receive/transmit errors. {@code percentBusUtilization} alone isn't a pass/fail signal (a
   * busy-but-healthy bus can sit at high utilization) -- these event counters are.
   */
  public static boolean regressed(BusReading previous, BusReading current) {
    return current.busOffCount() > previous.busOffCount()
        || current.txFullCount() > previous.txFullCount()
        || current.receiveErrorCount() > previous.receiveErrorCount()
        || current.transmitErrorCount() > previous.transmitErrorCount();
  }

  public static String describe(BusReading reading) {
    return String.format(
        "%s: util=%.1f%% busOff=%d txFull=%d rxErr=%d txErr=%d",
        reading.busName(),
        reading.percentBusUtilization(),
        reading.busOffCount(),
        reading.txFullCount(),
        reading.receiveErrorCount(),
        reading.transmitErrorCount());
  }
}
