package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    /* DECLARING CLASS VARIABLES */

    // motors Neo 550
    private final SparkMax mAlgaeLeftMotor;
    private final SparkMax mAlgaeRightMotor;

    // defining a variable motor speed
    private double mAlgaeMotorSpeed = 0.0;

    // configuration obj to configure the motor controller
    SparkFlexConfig config = new SparkFlexConfig();
    /* END CLASS VARIABLES */

    public AlgaeSubsystem() {

        // motors
        mAlgaeLeftMotor = new SparkMax(Constants.ALGAE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        mAlgaeRightMotor = new SparkMax(Constants.ALGAE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

        // what the motor should do when no input is given to the controller - in this case, brake
        config.idleMode(IdleMode.kBrake);
    }

    /* MOTOR FUNCTIONS */
    public void setAlgaeMotorSpeed(double pMotorSpeed) {
        mAlgaeMotorSpeed = pMotorSpeed;
    }

    // gets the intake motor speed
    public double getAlgaeMotorSpeed() {
        return mAlgaeMotorSpeed;
    }

    // stops the motors
    public void stopAlgaeMotors() {
        mAlgaeLeftMotor.set(0.0);
        mAlgaeRightMotor.set(0.0);
    }

    // actually sets the motor speed
    public void setAlgaeMotorsValue(double pMotorSpeed) {
        SmartDashboard.putNumber("Algae Intake: Motor Speed", pMotorSpeed);
        mAlgaeMotorSpeed = pMotorSpeed;
        mAlgaeLeftMotor.set(pMotorSpeed);
        mAlgaeRightMotor.set(-pMotorSpeed);
    }
    /* END MOTOR FUNCTIONS */

    // teleop command to intake/outtake Algae
    public Command handleAlgaeCommand(DoubleSupplier pSource) {
        return run(() -> {
            setAlgaeMotorSpeed(pSource.getAsDouble());
        });
    }
}
