// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
  public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = 22;
  public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = 29;

  // Elevator Limit Switch Constants
  public static final int TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = 1;
  public static final int BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL = 0;

  // Coral Reef Level Constants
  // where we want the bottom of coral payload to be at each reef level
  // TODO: Measure these for real
  public static final Distance CORAL_HEIGHT_L2 = Units.Inches.of(21.0);
  public static final Distance CORAL_HEIGHT_L3 = Units.Inches.of(36.0);
  public static final Distance CORAL_HEIGHT_L4 = Units.Inches.of(48.0);

  // ELEVATOR CONSTANTS
  // this is measured from bottom of coral payload to ground
  public static final double ELEVATOR_RESTING_POSITION_INCHES = 0; // TODO: FILL OUT!!!
  public static final Distance ELEVATOR_SPROCKET_RADIUS = Units.Inches.of(1.0);
  public static final Distance ELEVATOR_SPROCKET_DIAMETER = Units.Inches.of(2.0);
  public static final double ELEVATOR_MOTOR_GEAR_RATIO = 1.0 / 20.0;
  public static final LinearVelocity ELEVATOR_MAX_VELOCITY = Units.Inches.of(75.0).per(Units.Second);
  public static final LinearAcceleration ELEVATOR_MAX_ACCELERATION = Units.InchesPerSecond.of(50.0).per(Units.Second);

  // CLIMBER CONSTANTS
  public static final int CLIMBER_RELAY_PORT_ID = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // CORAL ARM SUBSYSTEM CONSTANTS
  public static final int CORAL_ARM_MOTOR_CAN_ID = 20;
  public static final AngularVelocity CORAL_ARM_MAX_VELOCITY = Units.DegreesPerSecond.of(90.0);
  public static final AngularAcceleration CORAL_ARM_MAX_ACCELERATION = Units.DegreesPerSecondPerSecond.of(45.0);

  // CORAL INTAKE SUBSYSTEM CONSTANTS
  public static final int CORAL_INTAKE_MOTOR_CAN_ID = 25;
  // beam break sensor
  public static final int CORAL_BEAM_BREAK_DIO_CHANNEL = 2;
  // auto Coral Intake/Outtake Speeds
  public static final double CORAL_INTAKE_SPEED = 0.0;
  public static final double CORAL_OUTTAKE_SPEED = 0.0;

  // Path Planner PID constants
  public static final PIDConstants PATH_FOLLOWING_PID_CONSTANTS_POSITIONAL = new PIDConstants(5.1275, 0.755);
  public static final PIDConstants PATH_FOLLOWING_PID_CONSTANTS_ROTATIONAL = new PIDConstants(1.985);
}
