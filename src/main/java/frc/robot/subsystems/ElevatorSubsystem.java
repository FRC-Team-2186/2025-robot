// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // SysId routine to model the behavior of our elevator subsystem
  private final SysIdRoutine mElevatorSysIdRoutine;

  // define sample size of sliding window to calculate velocity
  private final MedianFilter mElevatorVelocityFilter = new MedianFilter(32);

  // we need to define a PID control loop so we can monitor where the elevator is & where it needs to go
  private final ProfiledPIDController mElevatorPidController;

  // defining a variable motor speed
  private double mElevatorMotorSpeed = 0.0;

  // defining a boolean to control whether elevator Pid is enabled
  private boolean mElevatorPidControllerEnabled = false;

  // configuration obj to configure the motor controller
  SparkFlexConfig config = new SparkFlexConfig();

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

    // configuring the motor controllers
    // arg = how often we want the position to be measured
    config.signals.primaryEncoderPositionPeriodMs(5);
    // arg = sampling depth of encoder in bits
    config.absoluteEncoder.averageDepth(64);
    // position of encoder in "zero" position, aka resting position of elevator
    config.absoluteEncoder.zeroOffset(Constants.ELEVATOR_ABSOLUTE_ENCODER_OFFSET);
    // what the motors should do when no input is given to the controller - in this case, brake
    config.idleMode(IdleMode.kBrake);


  }

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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
