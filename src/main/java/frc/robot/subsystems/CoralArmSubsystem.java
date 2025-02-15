package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class CoralArmSubsystem extends SubsystemBase {
    /** SETTING UP CLASS VARIABLES */

    // Coral Motors
    private final SparkMax mCoralArmMotor;

    // Coral feedforward obj
    // initializing feedforward TODO: fill out
    private final ArmFeedforward mCoralArmFeedforward = new ArmFeedforward(0, 0, 0);

    // Coral arm encoder
    private RelativeEncoder mCoralEncoder;

    // pid for coral arm
    private final ProfiledPIDController mCoralPidController;

    // coral arm motor speed [0, 1]
    private double mCoralArmMotorSpeed = 0.0;

    // is pid enabled for the coral arm
    private boolean mCoralPidControllerEnabled = false;

    // SysId routine to model the behavior of our coral arm subsystem
    private final SysIdRoutine mCoralArmSysIdRoutine;

    // define sample size of sliding window to calculate velocity
    private final MedianFilter mCoralArmVelocityFilter = new MedianFilter(32);

    // configuration obj to configure the motor controller
    SparkFlexConfig config = new SparkFlexConfig();

    // rate limiter for arm
    private final SlewRateLimiter mCoralArmRateLimiter = new SlewRateLimiter(5);

    private final DigitalInput mCoralLimitSwitchBottom = 
      new DigitalInput(Constants.BOTTOM_CORAL_LIMIT_SWITCH_DIO_CHANNEL);
    private final DigitalInput mCoralLimitSwitchTop = 
      new DigitalInput(Constants.TOP_CORAL_LIMIT_SWITCH_DIO_CHANNEL);

    /* END CLASS VARIABLES */

    public CoralArmSubsystem() {

        // arm motor is a Neo 550
        mCoralArmMotor = new SparkMax(Constants.CORAL_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
        mCoralEncoder = mCoralArmMotor.getEncoder();

        // configuring the motor controller
        // arg = how often we want the position to be measured
        config.signals.primaryEncoderPositionPeriodMs(5);
        // arg = sampling depth of encoder in bits
        config.absoluteEncoder.averageDepth(64);
        // position of encoder in "zero" position, aka resting position of the arm
        config.absoluteEncoder.zeroOffset(Constants.CORAL_ARM_ABSOLUTE_ENCODER_OFFSET);
        // what the motors should do when no input is given to the controller - in this case, brake
        config.idleMode(IdleMode.kBrake);

        // initializing PID TODO: fill out
        mCoralPidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        mCoralPidController.setGoal(getEncoderPositionDegrees()); // TODO: tweak
        mCoralPidController.setTolerance(0.25); // TODO: tweak

        /** initializing the sysid routine * */
        mCoralArmSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                (volts) -> {
                    var voltage = volts.in(Units.Volts);
                    SmartDashboard.putNumber("Coral Arm Raw Voltage", voltage);
                    mCoralArmMotor.setVoltage(voltage);
                },
                this::handleSysIdLog,
                this));
        
        var tab = Shuffleboard.getTab("Arm");
        tab.addNumber("Setpoint", this::getCoralArmMotorSpeed);
        tab.addNumber(
            "Right Applied Voltage",
            () -> mCoralArmMotor.getAppliedOutput() * mCoralArmMotor.getBusVoltage());
        tab.addNumber("Encoder Position (Rotations)", () -> getArmPosition().in(Units.Rotations));
        tab.addNumber("Encoder Position (Degrees)", () -> getEncoderPositionDegrees());
        tab.addNumber(
            "Encoder Velocity (Rotations)", () -> getArmVelocity().in(Units.RotationsPerSecond));
        tab.addNumber("Encoder Velocity (Degrees)", () -> getArmVelocity().in(Units.DegreesPerSecond));
        tab.addBoolean("Is the Arm in Safe position to fire", () -> isArmSafe());
        tab.addBoolean("At Bottom", this::atBottom);
        tab.addBoolean("At Top", this::atTop);
        tab.add("PID Controller", mCoralPidController);
    }

    /* MOTOR FUNCTIONS */

    // gets the arm motor speed
    public double getCoralArmMotorSpeed() {
        return mCoralArmMotorSpeed;
    }

    // sets the coral arm motor speed
    public void setCoralArmMotorSpeed(double pMotorSpeed) {
        setCoralMotorValue(pMotorSpeed);
    }

    public void stopCoralArmMotor() {
        mCoralArmMotor.set(0);
    }

    // teleop command to move Arm up and down
    public Command moveCoralArmCommand(DoubleSupplier pSource) {
        return run(() -> {
        setCoralArmMotorSpeed(pSource.getAsDouble());
        });
    }

    // actually sets the speed on the motors & 
    private void setCoralMotorValue(double pMotorSpeed){
        if (pMotorSpeed > 0.0 && atTop()) {
        pMotorSpeed = 0.0;
        }
        if (pMotorSpeed < 0.0 && atBottom()) {
        pMotorSpeed = 0.0;
        }

        SmartDashboard.putNumber("Coral Arm: Motor Speed", pMotorSpeed);
        mCoralArmMotorSpeed = pMotorSpeed;
        mCoralArmMotor.set(pMotorSpeed);
    }
    /* END MOTOR FUNCTIONS */


    /* ENCODER FUNCTIONS */

    // tells us if the elevator is in it's resting position based on the bottom limit switch
    public boolean atBottom() {
        // return !mCoralLimitSwitchBottom.get();
        return false;
        // potential TODO: Ideally also determine this programatically
    }

    // tells us if the elevator is at its max height based on the top limit switch
    public boolean atTop() {
        // return mCoralLimitSwitchTop.get();
        return false;
        // potential TODO: Ideally also determine this programatically
    }

    // is Arm in a safe position (i.e. not at the top or bottom limits)
    public boolean isArmSafe() {
        return getEncoderPositionDegrees() <= Constants.CORAL_ARM_MAX_SAFE_ANGLE_DEGREES;
    }

    // gets position of encoder/arm in rotations -> returned as an Angle obj
    public Angle getArmPosition() {
        return Units.Rotations.of(mCoralEncoder.getPosition());
    }

    // gets position of encoder/arm in rotations -> returned as an Angle obj
    public double getEncoderPositionDegrees() {
        return getArmPosition().in(Units.Degrees);
    }

    // angular velocity of the arm
    public AngularVelocity getArmVelocity() {
        return Units.RotationsPerSecond.of(mCoralEncoder.getVelocity());
    }

    public Angle getDesiredAngle() {
        return Units.Degrees.of(mCoralPidController.getGoal().position);
      }
    
      public void setDesiredAngle(Angle pAngle) {
        mCoralPidController.setGoal(pAngle.in(Units.Degrees));
      }
    
      public boolean atDesiredAngle() {
        return mCoralPidController.atGoal();
    }

    public void usePidController() {
        mCoralPidControllerEnabled = true;
        mCoralPidController.reset(getEncoderPositionDegrees());
      }
    
      public void disablePidController() {
        mCoralPidControllerEnabled = false;
        setCoralMotorValue(0.0);
      }

    /* END ENCODER FUNCTIONS */

    public Command sysidQuasistatic(SysIdRoutine.Direction pDirection) {
        if (pDirection == SysIdRoutine.Direction.kForward) {
        return mCoralArmSysIdRoutine.quasistatic(pDirection).until(() -> atTop());
        } else {
        return mCoralArmSysIdRoutine.quasistatic(pDirection).until(() -> atBottom());
        }
    }

    public Command sysidDynamic(SysIdRoutine.Direction pDirection) {
        if (pDirection == SysIdRoutine.Direction.kForward) {
        return mCoralArmSysIdRoutine.dynamic(pDirection).until(() -> atTop());
        } else {
        return mCoralArmSysIdRoutine.dynamic(pDirection).until(() -> atBottom());
        }
    }

    private void handleSysIdLog(SysIdRoutineLog pLog) {
        var rightMotorVolts =
            Units.Volts.of(mCoralArmMotor.getAppliedOutput() * mCoralArmMotor.getBusVoltage());

        pLog.motor("coral arm")
            .voltage(rightMotorVolts.negate())
            .angularPosition(getArmPosition())
            .angularVelocity(getArmVelocity());
    }

}
