// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * Covers the no-absolute-encoder path: the modules on this robot are aligned by hand and zeroed
 * there, so the zeroing has to actually reach the IO layer.
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
  void absoluteEncodersAreOffSoTheHandZeroingPathIsTheLiveOne() {
    // This robot cannot connect the Thrifty encoders to its SystemCore. If someone flips this back
    // on without wiring them, ModuleIOSpark will read an unconnected analog channel and every module
    // will believe a constant heading, so the flag is worth asserting rather than assuming.
    assertFalse(
        Constants.Module.HAS_ABSOLUTE_ENCODERS,
        "absolute encoders are not wired on this robot; modules are zeroed by hand");
  }
}
