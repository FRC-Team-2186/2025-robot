package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final ArmFeedforward mCoralArmFeedforward;

    // Coral arm encoder
    private SparkAbsoluteEncoder mCoralEncoder;

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

    /* END CLASS VARIABLES */

    public CoralArmSubsystem() {

        // arm motor is a Neo 550
        mCoralArmMotor = new SparkMax(Constants.CORAL_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        // configuring the motor controller
        // arg = how often we want the position to be measured
        config.signals.primaryEncoderPositionPeriodMs(5);
        // arg = sampling depth of encoder in bits
        config.absoluteEncoder.averageDepth(64);
        // position of encoder in "zero" position, aka resting position of the arm
        config.absoluteEncoder.zeroOffset(Constants.CORAL_ARM_ABSOLUTE_ENCODER_OFFSET);
        // what the motors should do when no input is given to the controller - in this case, brake
        config.idleMode(IdleMode.kBrake);

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
    }

    /* MOTOR FUNCTIONS */

    // gets the arm motor speed
    public double getCoralArmMotorSpeed() {
        return mCoralArmMotorSpeed;
    }

    // sets the coral arm motor speed
    public void setCoralArmMotorSpeed(double pMotorSpeed) {
        mCoralArmMotorSpeed = pMotorSpeed;
    }

    public void stopCoralArmMotor() {
        mCoralArmMotor.set(0);
    }

    public Command moveCoralArmCommand(DoubleSupplier pSource) {
        return run(() -> {
        setCoralArmMotorSpeed(pSource.getAsDouble());
        });
    }

    private void setCoralMotorValue(double pMotorSpeed){
        if (pMotorSpeed > 0.0 && atTop()) {
        pMotorSpeed = 0.0;
        }
        if (pMotorSpeed < 0.0 && atBottom()) {
        pMotorSpeed = 0.0;
        }

        SmartDashboard.putNumber("Elevator: Motor Speed", pMotorSpeed);

        mLeftElevatorMotor.set(pMotorSpeed);
    }
  /* END MOTOR FUNCTIONS */

}
