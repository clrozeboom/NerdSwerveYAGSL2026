// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * Covers the hand-zeroing path -- still exercised even with a RioBridge supplying absolute
 * positions, since {@code TuningCommands.zeroModules} is still bound (for the "handled the robot,
 * need to re-align" case) regardless of {@link Constants.Module#HAS_ABSOLUTE_ENCODERS}.
 */
class ZeroModulesTest {

  /** Counts how many times it was told the module is pointing forward. */
  private static final class CountingIO implements ModuleIO {
    int zeroCalls = 0;

    @Override
    public void zeroTurnEncoder() {
      zeroCalls++;
    }
  }

  @Test
  void zeroingAModuleReachesTheIoLayer() {
    CountingIO io = new CountingIO();
    Module module = new Module(io, "TestModule", Constants.ModuleConfig.FRONT_LEFT);

    assertEquals(0, io.zeroCalls, "nothing should be zeroed before it is asked for");
    module.zeroTurnEncoder();
    assertEquals(1, io.zeroCalls, "zeroing the module should reach the IO layer");
  }

  @Test
  void absoluteEncodersComeFromTheRioBridgeNotADirectAnalogChannel() {
    // This robot still cannot connect the Thrifty encoders to its SystemCore directly -- the
    // RioBridge is what makes this true rather than false. If someone flips this back on without
    // also passing a real RioBridgeCan into ModuleIOSpark, readAbsolutePosition() falls back to
    // the hand-zeroed relative encoder instead of silently reading garbage (see its javadoc), but
    // that fallback defeats the point of turning this on, so the flag is still worth asserting
    // rather than assuming.
    assertTrue(
        Constants.Module.HAS_ABSOLUTE_ENCODERS,
        "absolute encoders are sourced from the RioBridge now -- see its javadoc before flipping"
            + " this back to false");
  }
}
