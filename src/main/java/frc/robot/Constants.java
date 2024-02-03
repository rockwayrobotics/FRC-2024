// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Constants for GamePads (taken from Driverstation)
  public static final class Gamepads {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
  }

  // CAN IDs for motor controllers 
  public static final class CAN {
    public static final int TALON_MOTOR = 1;
    public static final int SPARK_MOTOR = 2;
  }

  // Constants for digitals pins on the roboRIO 
  public static final class Digital {
    public static final int[] LEFT_DRIVE_ENCODER = { 2, 3 };
    public static final int[] RIGHT_DRIVE_ENCODER = { 4, 5 };

    public static final int LIMITSWITCH = 0; 
    public static final int COLOURMARK = 1; 
  }

  // Constants for the Drivebase/Robot Driving 
  public static final class Drive {
    public final static double ENCODER_PULSES_PER_REVOLUTION = 360;
    public final static double WHEEL_DIAMETER = 6;
    public final static double DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * Math.PI / ENCODER_PULSES_PER_REVOLUTION;

    public final static boolean LEFT_DRIVE_INVERTED = false;
    public final static boolean RIGHT_DRIVE_INVERTED = true;
  }

   // Constants for I2C ports
  public static final class I2C {
    public static final edu.wpi.first.wpilibj.I2C.Port COLOUR_SENSOR = edu.wpi.first.wpilibj.I2C.Port.kOnboard;
  }

  // Constants for LEDs 
  public static final class LED {
    public final static int LED_PWM = 0;
    public final static int LED_LENGTH = 10;

    public static enum modes {
      Green,
      Blue,
      oneSpace,
      singleRedDot
    }
}
}
