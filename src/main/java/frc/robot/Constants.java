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

  // Elevator Motor Constants
  public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = -1; // TODO: FILL OUT!!!
  public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = -1; // TODO: FILL OUT!!!

  // Elevator Limit Switch Constants
  public static final int TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = -1; // TODO: FILL OUT!!!
  public static final int BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = -1; // TODO: FILL OUT!!!

  // Elevator Encoder Constants
  public static final int ELEVATOR_ABSOLUTE_ENCODER_OFFSET = -1; // TODO: FILL OUT!!!

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
