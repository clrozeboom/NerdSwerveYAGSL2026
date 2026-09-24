// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Caption padding for the driver station display.
 *
 * <p>The display writes each line as {@code caption + " : " + value}, so every caption has to come
 * out the same width or the values do not line up into a column -- which is the only reason the
 * readings are worth glancing at while the robot is moving.
 */
class DriverDisplayTest {
  private static final int WIDTH = 10;

  @Test
  void everyCaptionIsExactlyTenCharacters() {
    for (String name :
        new String[] {"Auto", "Gyro", "SPARKs", "FrontLeft", "FrontRight", "BackLeft", "BackRight"}) {
      assertEquals(
          WIDTH, DriverDisplay.caption(name).length(), "caption \"" + name + "\" is not 10 wide");
    }
  }

  @Test
  void shortCaptionsArePaddedOnTheLeft() {
    assertEquals("      Auto", DriverDisplay.caption("Auto"));
    assertEquals("    SPARKs", DriverDisplay.caption("SPARKs"));
    assertEquals(" FrontLeft", DriverDisplay.caption("FrontLeft"));
  }

  @Test
  void aCaptionThatAlreadyFitsIsUnchanged() {
    // The longest one in use. If this ever needed padding the width would be wrong.
    assertEquals("FrontRight", DriverDisplay.caption("FrontRight"));
  }

  @Test
  void anOverlongCaptionIsTruncatedRatherThanBreakingTheColumn() {
    // Renaming a module is the realistic way this happens; one long row must not push its own
    // values out of line with the other three.
    assertEquals("FrontLeftM", DriverDisplay.caption("FrontLeftModule"));
    assertEquals(WIDTH, DriverDisplay.caption("a considerably longer caption").length());
  }
}
