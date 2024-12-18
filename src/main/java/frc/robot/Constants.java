// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int driverControllerPort = 0;
  }

  public static final class ArmConstants {
    public static final int motorPort = 6;
    public static final double speedFactor = 0.1;

    public static final double poskP = 0.01;
    public static final double poskI = 0;
    public static final double poskD = 0;

    public static final double velkP = 0.01;
    public static final double velkI = 0;
    public static final double velkD = 0;
  }

  public static final class EncoderConstants {
    public static final int channel1 = 0;
    public static final int channel2 = 1;
    public static final double ticksPerRev = 4096;
    public static double minAngle = 0;
    public static double maxAngle = 360; 
  }

}
