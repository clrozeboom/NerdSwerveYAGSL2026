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
 */
public class DiagnosticsRobot extends TimedRobot {
  private static final int RIOBRIDGE_BUS = CANBusMap.CAN_S1;
  private static final int DRIVETRAIN_BUS = CANBusMap.CAN_S0;
  private static final double PRINT_INTERVAL_SECONDS = 1.0;
  private static final double TIMESTAMP_CHECK_TIMEOUT_SECONDS = 10.0;

  private final RioBridgeCan rioBridgeCan = new RioBridgeCan(RIOBRIDGE_BUS, 32);

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

    previousDrivetrainReading = BusHealthMonitor.sample(DRIVETRAIN_BUS, "CAN_S0");
    previousRioBridgeReading = BusHealthMonitor.sample(RIOBRIDGE_BUS, "CAN_S1");
  }

  @Override
  public void close() {
    rioBridgeCan.close();
    super.close();
  }

  @Override
  public void robotPeriodic() {
    rioBridgeCan.poll();
    attitudeFramesSinceLastPrint += rioBridgeCan.drainAttitudeSamples().size();

    double now = Timer.getMonotonicTimestamp();
    if (now < nextPrintAt) {
      return;
    }
    nextPrintAt = now + PRINT_INTERVAL_SECONDS;

    BusHealthMonitor.BusReading drivetrain = BusHealthMonitor.sample(DRIVETRAIN_BUS, "CAN_S0");
    BusHealthMonitor.BusReading rioBridge = BusHealthMonitor.sample(RIOBRIDGE_BUS, "CAN_S1");

    System.out.println(
        BusHealthMonitor.describe(drivetrain)
            + flag(BusHealthMonitor.regressed(previousDrivetrainReading, drivetrain)));
    System.out.println(
        BusHealthMonitor.describe(rioBridge)
            + flag(BusHealthMonitor.regressed(previousRioBridgeReading, rioBridge)));
    System.out.printf(
        "  RioBridgeCan: attitudeFramesLastSecond=%d (expect ~%d at 100 Hz) overflowCount=%d%s%n",
        attitudeFramesSinceLastPrint,
        100,
        rioBridgeCan.overflowCount(),
        rioBridgeCan.overflowCount() > 0 ? "  <-- frames have been DROPPED, not just delayed" : "");

    attitudeFramesSinceLastPrint = 0;
    previousDrivetrainReading = drivetrain;
    previousRioBridgeReading = rioBridge;
  }

  private static String flag(boolean regressed) {
    return regressed ? "  <-- REGRESSED since last second" : "";
  }
}
