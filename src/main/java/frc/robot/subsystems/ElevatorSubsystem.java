// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
  /** SETTING UP CLASS VARIABLES */

  // Elevator Motors
  private final SparkMax mLeftElevatorMotor; // the "master"
  private final SparkMax mRightElevatorMotor;

  // Elevator FeedForward obj
  private final ElevatorFeedforward mElevatorFeedforward = new ElevatorFeedforward(0, 0, 0); // TODO: figure out what these constants should be

  /** Limit Switches for top & bottom of elevator
   * args = DIO channel for digital input
  */
  // private final DigitalInput mElevatorLimitSwitchBottom = 
  //   new DigitalInput(Constants.BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);
  // private final DigitalInput mElevatorLimitSwitchTop = 
  //   new DigitalInput(Constants.TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);

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

  // rate limiter for elevator
  private final SlewRateLimiter mElevatorRateLimiter = new SlewRateLimiter(5);

  /** END CLASS VARIABLES */

  
  /** Default Constructor that creates a new ElevatorSubsystem. 
   * Initializes the vars we declared in the class definition
  */
  public ElevatorSubsystem() {

    // These are both NEO Vortex motors
    // args = CAN ID of motor & MotorType
    mLeftElevatorMotor = new SparkMax(Constants.LEFT_ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
    mRightElevatorMotor = new SparkMax(Constants.RIGHT_ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

    // The left elevator motor has the encoder attached
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

    /** initializing the sysid routine 
     * 
    */
    mElevatorSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
              (volts) -> {
                var voltage = volts.in(Units.Volts);
                SmartDashboard.putNumber("Elevator Raw Voltage", voltage);
                mLeftElevatorMotor.setVoltage(voltage);
                mRightElevatorMotor.setVoltage(-voltage);
              },
              this::handleSysIdLog,
              this));


    /** initializing PID controller 
     * args: proportional coefficient, integral coefficient, derivative coefficient, and constraints for velocity & acceleration
     * TODO: fill out Pid values!!! */
    mElevatorPidController = 
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    mElevatorPidController.setGoal(getEncoderPositionMeters()); // TODO: idk if we need this?
    mElevatorPidController.setTolerance(0.05); // TODO: tweak

    var tab = Shuffleboard.getTab("Elevator");
    tab.addNumber("setpoint",this::getElevatorMotorSpeed);
    tab.addNumber(
      "Right Elevator Motor Applied Voltage",
      () -> mRightElevatorMotor.getAppliedOutput() * mRightElevatorMotor.getBusVoltage());
    tab.addNumber(
        "Left Elevator Motor Applied Voltage", 
        () -> mLeftElevatorMotor.getAppliedOutput() * mRightElevatorMotor.getBusVoltage());
    tab.addNumber("Encoder Position (Distance)", () -> getElevatorPosition().in(Units.Meters));
    tab.addNumber("Encoder Position (Meters)", () -> getEncoderPositionMeters());
    tab.addNumber(
        "Encoder Linear Velocity (Meters/Second)", () -> getElevatorVelocity().in(Units.MetersPerSecond));
    tab.addBoolean("Is the Elevator in a safe position", () -> isElevatorSafe());
    tab.addBoolean("At Bottom", this::atBottom);
    tab.addBoolean("At Top", this::atTop);
    // tab.add("Top Limit Switch", mElevatorLimitSwitchTop);
    // tab.add("Bottom Limit Switch", mElevatorLimitSwitchBottom);
    tab.add("PID Controller", mElevatorPidController);
    }


  /* MOTOR FUNCTIONS */

  // gets the elevator motor speed
  public double getElevatorMotorSpeed() {
    return mElevatorMotorSpeed;
  }

  // sets the elevator motor speed
  public void setElevatorMotorSpeed(double pMotorSpeed) {
    mElevatorMotorSpeed = pMotorSpeed;
  }

  public void stopElevatorMotors() {
    mLeftElevatorMotor.set(0);
    mRightElevatorMotor.set(0);
  }

  public Command moveElevatorDirectCommand(DoubleSupplier pSource) {
    return run(() -> {
      setElevatorMotorSpeed(pSource.getAsDouble());
    });
  }

  private void setElevatorMotorValues(double pMotorSpeed){
    if (pMotorSpeed > 0.0 && atTop()) {
      pMotorSpeed = 0.0;
    }
    if (pMotorSpeed < 0.0 && atBottom()) {
      pMotorSpeed = 0.0;
    }

    SmartDashboard.putNumber("Elevator: Motor Speed", pMotorSpeed);

    mLeftElevatorMotor.set(pMotorSpeed);
    mRightElevatorMotor.set(-pMotorSpeed);
  }
  /* END MOTOR FUNCTIONS */



  /* ENCODER FUNCTIONS */

  // tells us if the elevator is in it's resting position based on the bottom limit switch
  public boolean atBottom() {
    // return !mElevatorLimitSwitchBottom.get();
    return false;
    // potential TODO: Ideally also determine this programatically
  }

  // tells us if the elevator is at its max height based on the top limit switch
  public boolean atTop() {
    // return mElevatorLimitSwitchTop.get();
    return false;
    // potential TODO: Ideally also determine this programatically
  }

  // gets position of encoder/elevator in meters -> returned as a Distance obj
  public Distance getElevatorPosition() {
    // TODO: need to convert rotations to meters
    // meters per rotation * rotations
    return Units.Meters.of(mElevatorEncoder.getPosition());
  }

  // gets position of encoder/elevator in meters -> returned as a double
  public double getEncoderPositionMeters() {
    // TODO: need to convert rotations to meters
    // meters per rotation * rotations
    return getElevatorPosition().in(Units.Meters);
  }

  /* gets linear velocity of the elevator at a point in time (calculated over a sliding window) 
   * units = meters / second
  */
  public LinearVelocity getElevatorVelocity() {
    var velocityRotationsPerSecond = mElevatorEncoder.getVelocity();
    var filteredVelocity = mElevatorVelocityFilter.calculate(velocityRotationsPerSecond);
    // TODO -> calculate rotations per second * meters per rotation
    return Units.MetersPerSecond.of(filteredVelocity);
  } 

  // tells us if the elevator is at a safe height
  // Note: maybe this should be < slightly taller than L4?
  public boolean isElevatorSafe() {
    return getEncoderPositionMeters() <= Constants.ELEVATOR_MAX_SAFE_POSITION_INCHES;
  }

  // how far we need to raise the elevator to reach the coral level we define as the "goal"
  public Distance getDesiredDistance() {
    // TODO: verify units & conversions
    return Units.Meters.of(mElevatorPidController.getGoal().position);
  }

  // setting the goal state (in our case, the meters to the desired coral level)
  public void setDesiredDistance(Distance pDistance) {
    mElevatorPidController.setGoal(pDistance.in(Units.Meters));
  }

  // tells us if we're at the goal state (desired coral level)
  public boolean atDesiredDistance() {
    return mElevatorPidController.atGoal();
  }

  // initializes Elevator Pid controller & resets it to the position it's currently at w.r.t the elevator
  public void usePidController() {
    mElevatorPidControllerEnabled = true;
    mElevatorPidController.reset(getEncoderPositionMeters());
  }

  // disabled Pid for elevator and sets motor speed to 0
  public void disablePidController() {
    mElevatorPidControllerEnabled = false;
    mElevatorMotorSpeed = 0.0;
  }

  // resets the rate limiting factor for the elevator
  public void resetRateLimiter() {
    mElevatorRateLimiter.reset(0);
  }
  /* END ENCODER FUNCTIONS */


  /* MOTOR IDENTIFICATION TEST ROUTINES */
  // In this test, the elevator is gradually sped-up such that the voltage corresponding to acceleration is negligible (hence, “as if static”)
  public Command sysidQuasistatic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mElevatorSysIdRoutine.quasistatic(pDirection).until(() -> atTop());
    }
    else {
      return mElevatorSysIdRoutine.quasistatic(pDirection).until(() -> atBottom());
    }
  }

  // In this test, a constant ‘step voltage’ is given to the mechanism, so that the behavior while accelerating can be determined.
  public Command sysidDyamic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mElevatorSysIdRoutine.dynamic(pDirection).until(() -> atTop());
    }
    else {
      return mElevatorSysIdRoutine.dynamic(pDirection).until(() -> atBottom());
    }
  }

  // logs data from sysid test routines
  private void handleSysIdLog(SysIdRoutineLog pLog) {
    var rightElevatorMotorVolts = 
      Units.Volts.of(mRightElevatorMotor.getAppliedOutput() * mRightElevatorMotor.getBusVoltage());
    pLog.motor("Elevator")
      .voltage(rightElevatorMotorVolts.negate())
      .linearPosition(getElevatorPosition())
      .linearVelocity(getElevatorVelocity());
  }
  /* END MOTOR IDENTIFICATION TEST ROUTINES */



  @Override
  public void periodic() {
    // This method will be called once per scheduler run -> the "main" method for the elevator
    if (mElevatorPidControllerEnabled) {
      var setpoint = mElevatorPidController.getSetpoint();
      var pidOutput = mElevatorPidController.calculate(getEncoderPositionMeters());
      var feedforward = mElevatorFeedforward.calculate(
        getElevatorPosition().in(Units.Meters), getElevatorVelocity().in(Units.MetersPerSecond)
      );

      SmartDashboard.putNumber("Elevator: Raw Setpoint - Position", setpoint.position);
      SmartDashboard.putNumber("Elevator: Raw Setpoint - Velocity", setpoint.velocity);
      SmartDashboard.putNumber("Elevator: Raw PID Output", pidOutput);
      SmartDashboard.putNumber("Elevator: Raw FeedForward", feedforward);

      setElevatorMotorValues(pidOutput);
    }
    else {
      setElevatorMotorValues(mElevatorMotorSpeed); // TODO: tweak, speed * a constant
    }
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
