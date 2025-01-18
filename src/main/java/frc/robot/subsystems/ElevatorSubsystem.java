// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Setting Up Class Variables */

  // Elevator Motors
  private final SparkFlex mLeftElevatorMotor;
  private final SparkFlex mRightElevatorMotor;

  // Elevator FeedForward obj
  private final ElevatorFeedforward mElevatorFeedforward = new ElevatorFeedforward(0, 0, 0); // TODO: figure out what these constants should be

  /** Limit Switches for top & bottom of elevator
   * args = DIO channel for digital input
  */
  private final DigitalInput mElevatorLimitSwitchBottom = 
    new DigitalInput(Constants.BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);
  private final DigitalInput mElevatorLimitSwitchTop = 
    new DigitalInput(Constants.TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);


  /** TODO: decide whether we want the encoder to be set as relative or absolute & declare it here. 
   * declaring absolute for now
  */
  private SparkAbsoluteEncoder mElevatorEncoder;


  /** End Class Variables */

  
  /** Default Constructor that creates a new ElevatorSubsystem. 
   * Initializes the vars we declared in the class definition
  */
  public ElevatorSubsystem() {

    // These are both NEO Vortex motors
    // args = CAN ID of motor & MotorType
    mLeftElevatorMotor = new SparkFlex(Constants.LEFT_ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
    mRightElevatorMotor = new SparkFlex(Constants.RIGHT_ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

    // TODO: define l or r motor to be attached to encoder - using right for now
    mElevatorEncoder = mRightElevatorMotor.getAbsoluteEncoder();

  }

  
  /** Command to make elevator reach L2 of reef from any position
   * Can reuse in Auto 
  */
  public Command l2Command() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /** Command to make elevator reach L3 of reef from any position 
   * Can reuse in Auto
  */
  public Command l3Command() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /** Command to make elevator reach L4 of reef from any position 
   * Can reuse in Auto
  */
  public Command l4Command() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /** Command to make elevator return to starting (resting) position
   * Can reuse in Auto
   */
  public Command elevatorRestCommand(){
    return runOnce(
      () -> {
        // code here
      });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
