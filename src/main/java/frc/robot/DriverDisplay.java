// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import java.util.function.Supplier;
import org.wpilib.driverstation.DriverStationDisplay;
import org.wpilib.system.Timer;

/**
 * The few numbers worth reading off the driver station itself.
 *
 * <p>Everything here is also in the log and on NetworkTables, which is the right place to study it
 * afterwards. The point of this class is the other case: standing next to the robot during
 * bring-up with nothing open but the driver station, wanting to know which routine is selected and
 * whether the drivetrain agrees with what the wheels are visibly doing.
 *
 * <p>{@link DriverStationDisplay} is output only -- there is no widget to select anything from, so
 * the routine shown here is chosen through the driver station's own operating-mode list; see
 * {@link RobotContainer#publishOpModes()}.
 */
public final class DriverDisplay {
  /**
   * How often the display is rebuilt, in seconds.
   *
   * <p>{@code DriverStationDisplay.updateLines()} rate-limits itself to one write every 230 ms and
   * <em>discards</em> the lines it was given if it is called before that. Building them at the
   * 20 ms loop rate would therefore throw away nine batches in ten, so this gates on the way in
   * instead. Slightly slower than the display's own limit, to stay off the boundary.
   */
  private static final double UPDATE_PERIOD_SECS = 0.25;

  private final Drive drive;
  private final Supplier<String> selectedRoutine;
  private final Timer timer = new Timer();

  /**
   * @param drive the drivetrain to read
   * @param selectedRoutine what the driver station currently has selected for autonomous
   */
  public DriverDisplay(Drive drive, Supplier<String> selectedRoutine) {
    this.drive = drive;
    this.selectedRoutine = selectedRoutine;
    timer.start();
  }

  /** Rebuilds the display. Registered with the scheduler as a periodic sideload. */
  public void update() {
    if (!timer.hasElapsed(UPDATE_PERIOD_SECS)) {
      return;
    }
    timer.restart();

    DriverStationDisplay.addData("Auto", selectedRoutine.get());
    DriverStationDisplay.addData(
        "Gyro",
        "%+7.1f deg%s",
        drive.getRotation().getDegrees(),
        drive.isGyroConnected() ? "" : "  (integrated from wheels)");

    for (Module module : drive.getModules()) {
      // Heading first, then speed: during bring-up the question is almost always "is that wheel
      // pointing where the code thinks it is", and the answer is checkable by eye against the
      // module itself.
      DriverStationDisplay.addData(
          module.getName(),
          "%+7.1f deg  %+6.2f m/s%s",
          module.getAngle().getDegrees(),
          module.getVelocityMetersPerSec(),
          module.isConnected() ? "" : "  (no telem)");
    }

    // Worth a line of its own while the SPARKs are only reporting intermittently: without it a
    // module reading a flat 0.00 m/s looks like a stopped wheel rather than a silent controller.
    int connected = 0;
    for (Module module : drive.getModules()) {
      if (module.isConnected()) {
        connected++;
      }
    }
    DriverStationDisplay.addData("SPARKs", "%d/4 modules reporting", connected);

    DriverStationDisplay.updateLines();
  }
}
