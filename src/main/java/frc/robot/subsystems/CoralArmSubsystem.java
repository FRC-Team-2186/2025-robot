package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class CoralArmSubsystem extends SubsystemBase {
  /** SETTING UP CLASS VARIABLES */

  private static final AngleUnit POSITION_UNIT = Units.Radians;
  private static final AngularVelocityUnit VELOCITY_UNIT = Units.RadiansPerSecond;
  private static final AngularAccelerationUnit ACCELERATION_UNIT = Units.RadiansPerSecondPerSecond;

  // Coral Motors
  private final SparkMax mCoralArmMotor = new SparkMax(Constants.CORAL_ARM_MOTOR_CAN_ID,
      MotorType.kBrushless);

  // Coral feedforward obj
  private final ArmFeedforward mCoralArmFeedforward = new ArmFeedforward(0, 0.69, 0.2325, 1);

  // Coral arm encoder
  private final SparkAbsoluteEncoder mEncoder;

  // private final MedianFilter mEncoderFilter = new MedianFilter(1 << 7);

  // pid for coral arm
  // kP = 4.875 kI = 0.25 kD - 0.4
  private final ProfiledPIDController mCoralPidController = new ProfiledPIDController(2.0675, 0, 0,
      new Constraints(Constants.CORAL_ARM_MAX_VELOCITY.in(VELOCITY_UNIT),
          Constants.CORAL_ARM_MAX_ACCELERATION.in(Units.DegreesPerSecondPerSecond)));

  // SysId routine to model the behavior of our coral arm subsystem
  private final SysIdRoutine mCoralArmSysIdRoutine;

  /* END CLASS VARIABLES */

  // constructor
  public CoralArmSubsystem() {
    var sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kBrake).inverted(true);
    sparkConfig.absoluteEncoder.positionConversionFactor(1.0).velocityConversionFactor(1.0).inverted(true)
        .averageDepth(128);
    sparkConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);

    mCoralArmMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    mEncoder = mCoralArmMotor.getAbsoluteEncoder();

    mCoralArmSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Units.Volts.of(0.5).per(Units.Second), Units.Volts.of(4.0), Units.Seconds.of(10)),
        new SysIdRoutine.Mechanism(volts -> {
          mCoralArmMotor.setVoltage(volts);
        }, this::handleSysIdLog, this));

    mCoralPidController.setTolerance(Units.Degrees.of(0.5).in(POSITION_UNIT));
  }

  /* ENCODER FUNCTIONS */

  // gets position of encoder/arm in rotations -> returned as an Angle obj
  public Angle getArmPosition() {
    var degreesUnadjusted = mEncoder.getPosition() * 360.0;
    var degrees = ((degreesUnadjusted + 180.0) % 360.0) - 180.0;

    return Units.Degrees.of(degrees);
  }

  public double getArmPositionDegrees() {
    return getArmPosition().in(Units.Degrees);
  }

  public double getArmPositionRotations() {
    return getArmPosition().in(Units.Rotations);
  }

  public double getArmPositionRadians() {
    return getArmPosition().in(Units.Radians);
  }

  // angular velocity of the arm
  public AngularVelocity getArmVelocity() {
    var rpm = mEncoder.getVelocity();
    return Units.RotationsPerSecond.of(rpm / 60.0);
  }

  public double getArmVelocityDegrees() {
    return getArmVelocity().in(Units.DegreesPerSecond);
  }

  public double getArmVelocityRadiansPerSecond() {
    return getArmVelocity().in(Units.RadiansPerSecond);
  }

  public double getMotorVoltage() {
    return mCoralArmMotor.getAppliedOutput() * mCoralArmMotor.getBusVoltage();
  }

  public double getDesiredPosition() {
    return mCoralPidController.getSetpoint().position;
  }

  public double getDesiredVelocity() {
    return mCoralPidController.getSetpoint().velocity;
  }

  // execute command with PID based on the goal angle set (radians)
  private void executeWithPid() {
    var pidOutput = mCoralPidController.calculate(getArmPosition().in(Units.Radians));
    var feedforward = mCoralArmFeedforward.calculate(mCoralPidController.getSetpoint().position,
        mCoralPidController.getSetpoint().velocity);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FeedForward", feedforward);

    var motorOutput = pidOutput + feedforward;

    mCoralArmMotor.setVoltage(motorOutput);
  }

  private void pidBegin(Angle pGoal) {
    resetPidController();
    mCoralPidController.setGoal(pGoal.in(POSITION_UNIT));
  }

  private void pidEnd() {
    mCoralArmMotor.stopMotor();
    resetPidController();
  }

  // reset PID to ignore any previously set values
  public void resetPidController() {
    mCoralPidController.reset(getArmPosition().in(POSITION_UNIT), getArmVelocity().in(VELOCITY_UNIT));
  }

  /* END ENCODER FUNCTIONS */

  /* SYS ID FUNCTIONS */

  // In this test, the elevator is gradually sped-up such that the voltage
  // corresponding to acceleration is negligible (hence, “as if static”)
  public Command sysidQuasistatic(SysIdRoutine.Direction pDirection) {
    return mCoralArmSysIdRoutine.quasistatic(pDirection);
  }

  // In this test, a constant ‘step voltage’ is given to the mechanism, so that
  // the behavior while accelerating can be determined.
  public Command sysidDynamic(SysIdRoutine.Direction pDirection) {
    return mCoralArmSysIdRoutine.dynamic(pDirection);
  }

  // logging sys id routines
  private void handleSysIdLog(SysIdRoutineLog pLog) {
    var motorVolts = Units.Volts.of(mCoralArmMotor.getAppliedOutput() * mCoralArmMotor.getBusVoltage());

    pLog.motor("coral arm")
        .voltage(motorVolts.unaryMinus())
        .angularPosition(getArmPosition())
        .angularVelocity(getArmVelocity());
  }

  /* END SYS ID FUNCTIONS */

  /* COMMANDS */

  // teleop command to move Arm up and down
  public Command moveCoralArmCommand(DoubleSupplier pSource) {
    return run(() -> {
      mCoralArmMotor.set(pSource.getAsDouble());
    });
  }

  // move arm to specified position
  public Command moveCoralToPositionCommand(Angle pDesiredPosition) {
    return startRun(() -> {
      pidBegin(pDesiredPosition);
    }, () -> {
      executeWithPid();
    }).finallyDo(this::pidEnd);
  }

  /* END COMMANDS */

}
