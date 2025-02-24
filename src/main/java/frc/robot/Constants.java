// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.06;
  }
  public static final int neo_encoder_tick_per_rev = 42;
  public static final double startPos = 0;
  public static final double intakePos = 22;
  public final static double L1_scorePos = 14;
  public final static double L2_scorePos = 9.5;
  public final static double L3_scorePos = 15;
  public static final double MAX_SPEED = Units.feetToMeters(18.84);
}
