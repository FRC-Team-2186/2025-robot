// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

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

  // Elevator Motor Constants
  public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = 29; // TODO: FILL OUT!!!
  public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = 22; // TODO: FILL OUT!!!

  // Elevator Limit Switch Constants
  public static final int TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = 0; // TODO: FILL OUT!!!
  public static final int BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = 0; // TODO: FILL OUT!!!

  // Elevator Encoder Constants
  public static final int ELEVATOR_ABSOLUTE_ENCODER_OFFSET = 0; // TODO: FILL OUT!!!
  public static final double ELEVATOR_MAX_SAFE_POSITION_METERS = 0; // TODO: FILL OUT!!!

  // Coral Reef Level Constants
  // where we want the bottom of coral payload to be at each reef level
  public static final double L2_CORAL_METERS = 0.81; // TODO: ROUGH ESTIMATE - CONFIRM !!!
  public static final double L3_CORAL_METERS = 1.21; // TODO: ROUGH ESTIMATE - CONFIRM !!!
  public static final double L4_CORAL_METERS = 1.83; // TODO: ROUGH ESTIMATE - CONFIRM !!!

  // ELEVATOR CONSTANTS
  // this is measured from bottom of coral payload to ground
  public static final double ELEVATOR_RESTING_POSITION_METERS = 0; // TODO: FILL OUT!!!

  // CLIMBER CONSTANTS
  public static final int CLIMBER_RELAY_PORT_ID = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // CORAL ARM SUBSYSTEM CONSTANTS
  public static final int CORAL_ARM_MOTOR_CAN_ID = 0; // TODO: FILL OUT!!!
  public static final double CORAL_ARM_ABSOLUTE_ENCODER_OFFSET = 0; // TODO: FILL OUT!!!

  // CORAL INTAKE SUBSYSTEM CONSTANTS
  public static final int CORAL_INTAKE_MOTOR_CAN_ID = 0; // TODO: FILL OUT!!!
  // beam break sensor
  public static final int CORAL_BEAM_BREAK_DIO_CHANNEL = 0; // TODO: FILL OUT!!!
  // auto Coral Intake/Outtake Speeds
  public static final double CORAL_INTAKE_SPEED = 0.0;
  public static final double CORAL_OUTTAKE_SPEED = 0.0;

  // Path Planner PID constants
  public static final PIDConstants PATH_FOLLOWING_PID_CONSTANTS_POSITIONAL = new PIDConstants(5.1275, 0.755);
  public static final PIDConstants PATH_FOLLOWING_PID_CONSTANTS_ROTATIONAL = new PIDConstants(1.985);
}
