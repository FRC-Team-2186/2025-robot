package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

<<<<<<< HEAD
  // Elevator Motors
  private final SparkFlex mLeftElevatorMotor;
  private final SparkFlex mRightElevatorMotor;
=======
  private static final DistanceUnit POSITION_UNIT = Units.Inch;
  private static final LinearVelocityUnit VELOCITY_UNIT = Units.InchesPerSecond;
>>>>>>> 99d9c4d (feat: rewrite elevator code)

  private final SparkMax mLeftMotor = new SparkMax(Constants.LEFT_ELEVATOR_MOTOR_CAN_ID,
      MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(Constants.RIGHT_ELEVATOR_MOTOR_CAN_ID,
      MotorType.kBrushless);

  private final DigitalInput mTopLimitSwitch = new DigitalInput(Constants.TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);
  private final DigitalInput mBottomLimitSwitch = new DigitalInput(Constants.BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);

  private final SparkAbsoluteEncoder mEncoder = mLeftMotor.getAbsoluteEncoder();

  private final ProfiledPIDController mPidController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new Constraints(Constants.ELEVATOR_MAX_VELOCITY_METERS_PER_SECOND, Constants.ELEVATOR_MAX_ACCELERATION));
  private final ElevatorFeedforward mFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);

  private final SysIdRoutine mSysIdRoutine;

  public ElevatorSubsystem() {
    var sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kBrake);

    mLeftMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    mRightMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

<<<<<<< HEAD
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
     * args: proportional coefficient, integral coefficiet, derivative coefficient, and constraints for velocity & acceleration
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
    tab.add("Top Limit Switch", mElevatorLimitSwitchTop);
    tab.add("Bottom Limit Switch", mElevatorLimitSwitchBottom);
    tab.add("PID Controller", mElevatorPidController);
    }


  /* MOTOR FUNCTIONS */

  // gets the elevator motor speed
  public double getElevatorMotorSpeed() {
    return mElevatorMotorSpeed;
=======
    mSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(volts -> {
      mLeftMotor.setVoltage(volts);
      mRightMotor.setVoltage(volts.unaryMinus());
    }, this::handleSysIdLog, this));
>>>>>>> 99d9c4d (feat: rewrite elevator code)
  }

  public Distance getPosition() {
    var conversionFactor = (2.0 * Math.PI * Constants.ELEVATOR_SPROCKET_RADIUS.in(POSITION_UNIT))
        / Constants.ELEVATOR_MOTOR_GEAR_RATIO;
    var inches = mEncoder.getPosition() * conversionFactor;

    return POSITION_UNIT.of(inches);
  }

  public LinearVelocity getVelocity() {
    var conversionFactor = (2.0 * Math.PI * Constants.ELEVATOR_SPROCKET_RADIUS.in(POSITION_UNIT))
        / (Constants.ELEVATOR_MOTOR_GEAR_RATIO * 60.0);
    var ips = mEncoder.getVelocity() * conversionFactor;

    return VELOCITY_UNIT.of(ips);
  }

<<<<<<< HEAD
  public void setElevatorMotorValues(double pMotorSpeed){
    if (pMotorSpeed > 0.0 && atTop()) {
      pMotorSpeed = 0.0;
    }
    if (pMotorSpeed < 0.0 && atBottom()) {
      pMotorSpeed = 0.0;
    }

    SmartDashboard.putNumber("Elevator: Raw Voltage", pMotorSpeed);

    mRightElevatorMotor.setVoltage(-pMotorSpeed);
    mLeftElevatorMotor.setVoltage(pMotorSpeed);
  }
  /* END MOTOR FUNCTIONS */



  /* ENCODER FUNCTIONS */

  // tells us if the elevator is in it's resting position bsed on the bottom limit switch
  public boolean atBottom() {
    return !mElevatorLimitSwitchBottom.get();
  }

  // tells us if the elevator is at its max height based on the top limit switch
  public boolean atTop() {
    return mElevatorLimitSwitchTop.get();
=======
  public boolean atTop() {
    return mTopLimitSwitch.get();
>>>>>>> 99d9c4d (feat: rewrite elevator code)
  }

  public boolean atBottom() {
    return mBottomLimitSwitch.get();
  }

  public void resetPidController() {
    mPidController.reset(getPosition().in(POSITION_UNIT), getVelocity().in(VELOCITY_UNIT));
  }

  public Command directCommand(DoubleSupplier pSpeed) {
    return run(() -> {
      setMotorValues(pSpeed.getAsDouble());
    });
  }

  public Command moveToPositionCommand(Distance pDesiredPosition) {
    return startRun(() -> {
      resetPidController();
      mPidController.setGoal(pDesiredPosition.in(POSITION_UNIT));
    }, () -> {
      executeWithPid();
    }).until(mPidController::atGoal).finallyDo(() -> setMotorValues(0));
  }

  // In this test, the elevator is gradually sped-up such that the voltage
  // corresponding to acceleration is negligible (hence, “as if static”)
  public Command sysidQuasistatic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mSysIdRoutine.quasistatic(pDirection).until(() -> atTop());
    } else {
      return mSysIdRoutine.quasistatic(pDirection).until(() -> atBottom());
    }
  }

  // In this test, a constant ‘step voltage’ is given to the mechanism, so that
  // the behavior while accelerating can be determined.
  public Command sysidDyamic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atTop());
    } else {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atBottom());
    }
  }

  private void executeWithPid() {
    var pidOutput = mPidController.calculate(getPosition().in(POSITION_UNIT));
    var feedforward = mFeedforward.calculate(mPidController.getSetpoint().velocity);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FeedForward", feedforward);

    var motorOutput = (pidOutput * RobotController.getBatteryVoltage()) + feedforward;

    setMotorVoltage(Units.Volts.of(motorOutput));
  }

  private void setMotorValues(double pValue) {
    setMotorVoltage(Units.Volts.of(pValue * RobotController.getBatteryVoltage()));
  }

  private void setMotorVoltage(Voltage pVolts) {
    if (pVolts.gt(Units.Volts.zero()) && atTop()) {
      pVolts = Units.Volts.zero();
    }
    if (pVolts.lt(Units.Volts.zero()) && atBottom()) {
      pVolts = Units.Volts.zero();
    }

    mLeftMotor.setVoltage(pVolts);
    mRightMotor.setVoltage(pVolts.unaryMinus());
  }

  private void handleSysIdLog(SysIdRoutineLog pLog) {
    var leftMotorVolts = mLeftMotor.getAppliedOutput() * mLeftMotor.getBusVoltage();

<<<<<<< HEAD


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
      setElevatorMotorValues(mElevatorMotorSpeed * 12.0); // TODO: tweak
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
=======
    pLog.motor("Elevator").voltage(Units.Volts.of(leftMotorVolts)).linearPosition(getPosition())
        .linearVelocity(getVelocity());
>>>>>>> 99d9c4d (feat: rewrite elevator code)
  }
}
