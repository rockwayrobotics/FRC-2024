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
  public static final class Gamepads {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
  }

  public static final class CAN {
    public static final int LEFT_DRIVE_MOTOR_R = 1;
    public static final int LEFT_DRIVE_MOTOR_F = 2;
    public static final int RIGHT_DRIVE_MOTOR_R = 3;
    public static final int RIGHT_DRIVE_MOTOR_F = 4;
    public static final int LEFT_INTAKE = 5; // brushed
    public static final int RIGHT_INTAKE = 6; // brushed one opposite direction
    public static final int BELT = 7; // indexer
    public static final int GEAR = 8; // angle
    public static final int LEFT_FLYWHEEL = 9; // negative on one flywheel?
    public static final int RIGHT_FLYWHEEL = 10; // negative on one flywheel?
    public static final int CLIMB = 11;
  }

  public static final class Digital {
    public static final int[] LEFT_DRIVE_ENCODER = { 0, 1 };
    public static final int[] RIGHT_DRIVE_ENCODER = { 2, 3 };
  }

  public static final class Drive {
    public final static double ENCODER_PULSES_PER_REVOLUTION = 360;
    public final static double WHEEL_DIAMETER = 6;
    public final static double DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * Math.PI / ENCODER_PULSES_PER_REVOLUTION;

    public final static boolean LEFT_DRIVE_INVERTED = true;
    public final static boolean RIGHT_DRIVE_INVERTED = false;

    public final static double rotation_kP = 0.3;
  }

  // Constants for LEDs
  public static final class LED {
    public final static int LED_PWM = 0;
    public final static int LED_LENGTH = 60;

    public static enum modes {
      Rainbow,
      Red,
      Green,
      Blue,
      Purple,
      Orange,
      Yellow,
      BreathingYellow,
      BreathingMagenta,
      badApple,
      heatGradient,
      imageLoop,
    }
  }

  public static final class Shooter {
    public static enum ScoringMode {
      SPEAKER,
      AMP,
      TRAP,
    }
  }
}
