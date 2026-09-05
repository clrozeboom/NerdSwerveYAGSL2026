package frc.robot.subsystems.drive.riobridge.diagnostics;

import frc.robot.subsystems.drive.riobridge.RioBridgeCan;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.system.Timer;

/**
 * A throwaway robot program for the two Core-side items in the root RioBridge README's "to
 * verify" list that need a live CAN bus: timestamp units (item 1) and RX headroom under load
 * (item 3). Not part of the real robot code -- see RioBridge's docs/hardware-verification.md for
 * how to deploy this in place of the real robot code for one bench session, then revert to {@code
 * frc.robot.Main}.
 *
 * <p>Runs {@link TimestampUnitsCheck} once at startup, then continuously polls a {@link
 * RioBridgeCan} and both buses' {@link BusHealthMonitor} counters, printing a line per bus per
 * second and flagging any regression.
 *
 * <p><b>{@code attitudeFramesLastSecond} caps at ~50, not ~100, and that's correct.</b> This
 * class runs at {@code TimedRobot}'s 50 Hz default period, and {@link RioBridgeCan} has no
 * per-sample buffering (see its class javadoc) -- {@code readPacketLatest} only ever returns
 * whichever single packet is most recent at the moment of the call. Polling a 100 Hz source at
 * 50 Hz structurally cannot observe more than 50 distinct samples/sec no matter how well
 * everything else is working; confirmed on real hardware landing right at that ceiling
 * (consistently ~50-51/sec) once the payload-marshaling fix below actually worked.
 *
 * <p>{@link #rioBridgeCan} is still constructed after the steps above run, not as a field
 * initializer, matching the previous (stream-session-based) version of this class -- that
 * ordering was originally needed to avoid a real buffer-overflow crash, which doesn't apply to
 * this {@code readPacketLatest}-based implementation (see {@code RioBridgeCan}'s class javadoc:
 * there's no buffered session here to overflow), but there's no reason to reintroduce a field
 * initializer either.
 */
public class DiagnosticsRobot extends TimedRobot {
  private static final int RIOBRIDGE_BUS = CANBusMap.CAN_S1;
  private static final int DRIVETRAIN_BUS = CANBusMap.CAN_S0;
  private static final double PRINT_INTERVAL_SECONDS = 1.0;
  private static final double TIMESTAMP_CHECK_TIMEOUT_SECONDS = 10.0;

  private final RioBridgeCan rioBridgeCan;

  private BusHealthMonitor.BusReading previousDrivetrainReading;
  private BusHealthMonitor.BusReading previousRioBridgeReading;
  private double nextPrintAt = 0;
  private int attitudeFramesSinceLastPrint = 0;

  public DiagnosticsRobot() {
    System.out.println("=== TimestampUnitsCheck: collecting Status frames on CAN_S1 ===");
    TimestampUnitsCheck.Result result =
        TimestampUnitsCheck.run(RIOBRIDGE_BUS, TIMESTAMP_CHECK_TIMEOUT_SECONDS);
    System.out.printf(
        "  wall-clock elapsed=%.3fs, raw timestamp delta=%d, secondsPerUnit=%g%n",
        result.wallClockDeltaSeconds(), result.rawTimestampDelta(), result.secondsPerUnit());
    System.out.println("  " + result.verdict());
    if (!result.succeeded()) {
      System.out.println(
          "  Bus health monitoring below will still run, but confirm the RioBridge is actually"
              + " transmitting before trusting the numbers.");
    }
    System.out.println("=== Starting continuous bus health monitoring (item 3) ===");
    System.out.println(
        "Command the drivetrain (or otherwise load bus 0) during this run -- an idle bus 0"
            + " doesn't exercise the shared SPI master this is checking.");

    rioBridgeCan = new RioBridgeCan(RIOBRIDGE_BUS);

    previousDrivetrainReading = sampleSafely(DRIVETRAIN_BUS, "CAN_S0");
    previousRioBridgeReading = sampleSafely(RIOBRIDGE_BUS, "CAN_S1");
  }

  @Override
  public void close() {
    rioBridgeCan.close();
    super.close();
  }

  @Override
  public void robotPeriodic() {
    rioBridgeCan.poll();
    attitudeFramesSinceLastPrint += rioBridgeCan.drainNewAttitudeSampleCount();

    double now = Timer.getMonotonicTimestamp();
    if (now < nextPrintAt) {
      return;
    }
    nextPrintAt = now + PRINT_INTERVAL_SECONDS;

    BusHealthMonitor.BusReading drivetrain = sampleSafely(DRIVETRAIN_BUS, "CAN_S0");
    BusHealthMonitor.BusReading rioBridge = sampleSafely(RIOBRIDGE_BUS, "CAN_S1");

    printReading(drivetrain, previousDrivetrainReading);
    printReading(rioBridge, previousRioBridgeReading);
    System.out.printf(
        "  RioBridgeCan: attitudeFramesLastSecond=%d (expect ~%d -- this loop's 50 Hz default"
            + " period, not the Attitude frame's 100 Hz send rate: readPacketLatest has no"
            + " buffering, so a 50 Hz poll of a 100 Hz source structurally can't observe more"
            + " than 50 distinct samples/sec) overflowCount=%d"
            + " malformedFrameCount=%d%s%n",
        attitudeFramesSinceLastPrint,
        50,
        rioBridgeCan.overflowCount(),
        rioBridgeCan.malformedFrameCount(),
        rioBridgeCan.overflowCount() > 0 || rioBridgeCan.malformedFrameCount() > 0
            ? "  <-- frames have been DROPPED or discarded, not just delayed"
            : "");
    if (rioBridgeCan.malformedFrameCount() > 0) {
      System.out.println(
          "    last malformed frame: " + rioBridgeCan.lastMalformedFrameDescription());
    }

    attitudeFramesSinceLastPrint = 0;
    previousDrivetrainReading = drivetrain;
    previousRioBridgeReading = rioBridge;
  }

  private static void printReading(
      BusHealthMonitor.BusReading current, BusHealthMonitor.BusReading previous) {
    if (current == null) {
      return; // sampleSafely already explained why.
    }
    boolean regressed = previous != null && BusHealthMonitor.regressed(previous, current);
    System.out.println(BusHealthMonitor.describe(current) + flag(regressed));
  }

  /**
   * {@code CANJNI.getCANStatus} can throw at the HAL layer for a bus this process has never
   * otherwise touched -- confirmed on real hardware for the drivetrain bus in a run of exactly
   * this class, where nothing here ever constructs a device or opens a session on it (that
   * happens in the real robot code this class temporarily replaces, not here). Its SocketCAN
   * interface simply not being up yet on the Core is the other plausible cause. Reported instead
   * of crashing the whole diagnostic and losing visibility into the other bus -- which matters
   * most exactly when it's the drivetrain bus that's failing, since the RioBridge bus is the one
   * this class's own {@link RioBridgeCan} session already guarantees is touched.
   */
  private static BusHealthMonitor.BusReading sampleSafely(int bus, String busName) {
    try {
      return BusHealthMonitor.sample(bus, busName);
    } catch (RuntimeException e) {
      System.out.println(
          busName
              + ": status unavailable ("
              + e.getMessage()
              + "). Likely means this bus isn't brought up on the Core yet, or nothing in this"
              + " process has used it -- doesn't necessarily mean anything is wrong with the bus"
              + " itself.");
      return null;
    }
  }

  private static String flag(boolean regressed) {
    return regressed ? "  <-- REGRESSED since last second" : "";
  }
}
