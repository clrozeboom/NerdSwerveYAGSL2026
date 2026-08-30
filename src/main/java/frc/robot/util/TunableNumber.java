// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be edited live from the dashboard during bring-up, and that hard-codes itself
 * once tuning is switched off.
 *
 * <p>This is the piece that makes gain tuning practical: change a kP on the dashboard, watch the
 * step response, change it again — no redeploy between attempts. When
 * {@link Constants#TUNING_MODE} is false the dashboard entry is never created and {@link #get()}
 * returns the compiled-in default, so nothing on a competition robot depends on a dashboard value
 * somebody forgot to set.
 *
 * <p>It is built on AdvantageKit's {@link LoggedNetworkNumber} rather than a bare NetworkTables
 * entry so the value lands in the log. A replayed log then re-runs with the gains that were actually
 * in effect, which is the whole point of replay.
 */
public class TunableNumber {
  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;

  /** Last value each caller of {@link #hasChanged(int)} saw, keyed by that caller's id. */
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Creates a tunable number.
   *
   * @param key dashboard key, conventionally "Tuning/Subsystem/Gain"
   * @param defaultValue the compiled-in value, used as-is when tuning mode is off
   */
  public TunableNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.dashboardNumber =
        Constants.TUNING_MODE ? new LoggedNetworkNumber(key, defaultValue) : null;
  }

  /** The current value: the dashboard's in tuning mode, the compiled-in default otherwise. */
  public double get() {
    return dashboardNumber == null ? defaultValue : dashboardNumber.get();
  }

  /** The dashboard key, for logging alongside whatever this gain drives. */
  public String getKey() {
    return key;
  }

  /**
   * Reports whether the value has changed since this caller last asked.
   *
   * <p>Pushing gains to a motor controller is not free — on a SPARK MAX it is a CAN transaction — so
   * callers use this to reconfigure only on an actual edit rather than every loop.
   *
   * @param id a stable id unique to the caller, conventionally {@code hashCode()} of the calling
   *     object so that each module tracks its own edits independently
   * @return true the first time it is called, and thereafter only after the value changes
   */
  public boolean hasChanged(int id) {
    double current = get();
    Double last = lastHasChangedValues.get(id);
    if (last == null || current != last) {
      lastHasChangedValues.put(id, current);
      return true;
    }
    return false;
  }

  /** True if any of the given numbers has changed for this caller. Evaluates all of them. */
  public static boolean anyChanged(int id, TunableNumber... numbers) {
    boolean changed = false;
    for (TunableNumber number : numbers) {
      changed |= number.hasChanged(id);
    }
    return changed;
  }
}
