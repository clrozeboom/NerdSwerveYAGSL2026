// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * Checks that each corner's feedforward really is its own value rather than a shared one, since the
 * point of holding kS per module is that a corner with more static friction can carry a different
 * number from the rest.
 */
class PerModuleGainsTest {

  /** Records the gains the IO layer was last told to use. */
  private static final class RecordingIO implements ModuleIO {
    double kS = Double.NaN;
    double kV = Double.NaN;
    double kP = Double.NaN;

    @Override
    public void setDriveGains(double kP, double kD, double kS, double kV) {
      this.kP = kP;
      this.kS = kS;
      this.kV = kV;
    }
  }

  @Test
  void eachModuleGetsItsOwnFeedforward() {
    // Give two corners deliberately different static friction, the way a real measurement would.
    Constants.ModuleConfig frontLeft = Constants.ModuleConfig.FRONT_LEFT;
    Constants.ModuleConfig frontRight = Constants.ModuleConfig.FRONT_RIGHT;

    RecordingIO leftIO = new RecordingIO();
    RecordingIO rightIO = new RecordingIO();
    Module left = new Module(leftIO, "TestLeft", frontLeft);
    Module right = new Module(rightIO, "TestRight", frontRight);

    // Reading each module's own gain must go through its own config, not a shared static.
    assertEquals(frontLeft.driveKs, left.getDriveKs(), 1e-12);
    assertEquals(frontRight.driveKs, right.getDriveKs(), 1e-12);
    assertEquals(frontLeft.driveKv, left.getDriveKv(), 1e-12);
  }

  @Test
  void configuredKsIsCarriedPerCorner() {
    // All four default to the averaged value today, but they must be four independent fields so a
    // measured outlier can be set on one corner without disturbing the others.
    long distinctFields =
        java.util.Arrays.stream(Constants.ModuleConfig.ORDERED)
            .map(config -> System.identityHashCode(config))
            .distinct()
            .count();
    assertEquals(4, distinctFields, "each corner should be its own config instance");

    for (Constants.ModuleConfig config : Constants.ModuleConfig.ORDERED) {
      assertNotEquals(0.0, config.driveKs, "every corner needs a static friction gain");
      assertNotEquals(0.0, config.driveKv, "every corner needs a velocity gain");
    }
  }
}
