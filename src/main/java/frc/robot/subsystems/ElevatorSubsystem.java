package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
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

  private static final DistanceUnit POSITION_UNIT = Units.Inch;
  private static final LinearVelocityUnit VELOCITY_UNIT = Units.InchesPerSecond;

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

    mSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(volts -> {
      mLeftMotor.setVoltage(volts);
      mRightMotor.setVoltage(volts.unaryMinus());
    }, this::handleSysIdLog, this));
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

  public boolean atTop() {
    return mTopLimitSwitch.get();
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

    pLog.motor("Elevator").voltage(Units.Volts.of(leftMotorVolts)).linearPosition(getPosition())
        .linearVelocity(getVelocity());
  }
}
