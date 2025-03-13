package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
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
  private static final DistanceUnit POSITION_UNIT = Units.Inches;
  private static final LinearVelocityUnit VELOCITY_UNIT = Units.InchesPerSecond;
  private static final LinearAccelerationUnit ACCELERATION_UNIT = Units.InchesPerSecond.per(Units.Second);

  private static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new Constraints(
      Constants.ELEVATOR_MAX_VELOCITY.in(VELOCITY_UNIT), Constants.ELEVATOR_MAX_ACCELERATION.in(ACCELERATION_UNIT));

  private final SparkMax mLeftMotor = new SparkMax(Constants.LEFT_ELEVATOR_MOTOR_CAN_ID,
      MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(Constants.RIGHT_ELEVATOR_MOTOR_CAN_ID,
      MotorType.kBrushless);

  @Logged
  private final DigitalInput mBottomLimitSwitch = new DigitalInput(Constants.BOTTOM_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);
  @Logged
  private final DigitalInput mTopLimitSwitch = new DigitalInput(Constants.TOP_ELEVATOR_LIMIT_SWITCH_DIO_CHANNEL);

  private final RelativeEncoder mRelativeEncoder = mLeftMotor.getAlternateEncoder();

  @Logged
  private final ProfiledPIDController mPidController = new ProfiledPIDController(2.25, 0.0, 0.0,
      MOTION_CONSTRAINTS);

  // Known good-ish: kS = 0.0, kG = 0.33125, kV = 0.2255, kA = 0.5

  @Logged
  private final ElevatorFeedforward mFeedforward = new ElevatorFeedforward(0.26, 0.099847, 0.2225, 0.45);
  private final SysIdRoutine mSysIdRoutine;

  public ElevatorSubsystem() {
    var leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.alternateEncoder.setSparkMaxDataPortConfig();
    // leftConfig.smartCurrentLimit(6, 6);
    mLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    var rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    // rightConfig.smartCurrentLimit(6, 6);
    mRightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    mSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Units.Volts.of(0.25).per(Units.Second), Units.Volts.of(7.0), Units.Seconds.of(30.0)),
        new SysIdRoutine.Mechanism(volts -> {
          mLeftMotor.setVoltage(volts);
          mRightMotor.setVoltage(volts.unaryMinus());
        }, this::handleSysIdLog, this));

    mRelativeEncoder.setPosition(0.0);
    mPidController.disableContinuousInput();
    mPidController.setTolerance(0.06);

    // setDefaultCommand(moveToCurrentGoalCommand());
  }

  public Distance getPosition() {
    var conversionFactor = 2.0 * Math.PI * Constants.ELEVATOR_SPROCKET_RADIUS.in(POSITION_UNIT);
    var rotations = mRelativeEncoder.getPosition();
    var inches = rotations * conversionFactor;
    if (inches > 500) {
      System.out
          .println("ERROR: Relative Encoder is acting up again: inches = " + inches + ", rotations = " + rotations);
    }

    return Units.Inches.of(inches);
  }

  public LinearVelocity getVelocity() {
    var conversionFactor = (2.0 * Math.PI * Constants.ELEVATOR_SPROCKET_RADIUS.in(POSITION_UNIT));
    var ips = (mRelativeEncoder.getVelocity() * conversionFactor) / 60.0;

    return Units.InchesPerSecond.of(ips);
  }

  public double getPositionInches() {
    return getPosition().in(Units.Inches);
  }

  public double getVelocityInchesPerSecond() {
    return getVelocity().in(Units.InchesPerSecond);
  }

  public double getDesiredPosition() {
    return mPidController.getSetpoint().position;
  }

  public double getDesiredVelocity() {
    return mPidController.getSetpoint().velocity;
  }

  public boolean atBottom() {
    return !mBottomLimitSwitch.get();
  }

  public boolean atTop() {
    return mTopLimitSwitch.get();
  }

  public double getLeftMotorVoltage() {
    return mLeftMotor.getAppliedOutput() * mLeftMotor.getBusVoltage();
  }

  public Command directCommand(DoubleSupplier pSpeed) {
    return run(() -> {
      var voltage = pSpeed.getAsDouble() * RobotController.getBatteryVoltage();
      setMotorVoltage(Volts.of(voltage));
    }).finallyDo(() -> setMotorVoltage(Volts.zero()));
  }

  public Command moveToHeightCommand(Distance pDesiredHeight) {
    return startRun(() -> pidBegin(pDesiredHeight), () -> executeWithPid(pDesiredHeight))
        .until(() -> mPidController.atGoal()).finallyDo(this::pidEnd);
  }

  public Command homeCommand() {
    return directCommand(() -> -0.25).until(this::atBottom);
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
  public Command sysidDynamic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atTop());
    } else {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atBottom());
    }
  }

  private TrapezoidProfile.State getMotionProfileState() {
    double position = getPosition().in(Units.Inches);
    double velocity = getVelocity().in(VELOCITY_UNIT);
    return new State(position, velocity);
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

  private void stopMotors() {
    setMotorVoltage(Volts.zero());
  }

  private void pidBegin(Distance pGoal) {
    var newState = getMotionProfileState();

    mPidController.reset(newState);
    mPidController.setGoal(pGoal.in(POSITION_UNIT));
  }

  private void pidEnd() {
    stopMotors();

    mPidController.reset(getPosition().in(POSITION_UNIT));
  }

  private void executeWithPid(Distance pGoal) {
    var feedbackVoltage = mPidController.calculate(getPosition().in(POSITION_UNIT));
    var feedforwardVoltage = mFeedforward.calculate(mPidController.getSetpoint().velocity);

    SmartDashboard.putNumber("PID Output", feedbackVoltage);
    SmartDashboard.putNumber("FeedForward", feedforwardVoltage);
    SmartDashboard.putNumber("Goal", pGoal.in(POSITION_UNIT));

    var volts = Volts.of(feedforwardVoltage + feedbackVoltage);

    setMotorVoltage(volts);
  }

  private void handleSysIdLog(SysIdRoutineLog pLog) {
    var leftMotorVolts = mLeftMotor.getAppliedOutput() * mLeftMotor.getBusVoltage();

    pLog.motor("Elevator").voltage(Units.Volts.of(leftMotorVolts)).linearPosition(getPosition())
        .linearVelocity(getVelocity());
  }

  @Override
  public void periodic() {
    if (atBottom()) {
      mRelativeEncoder.setPosition(0.0);
    }
  }
}
