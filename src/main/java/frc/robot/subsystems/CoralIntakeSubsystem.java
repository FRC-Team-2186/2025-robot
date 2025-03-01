package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class CoralIntakeSubsystem extends SubsystemBase {

    /** SETTING UP CLASS VARIABLES */

    // intake motor is a Neo 550
    private final SparkMax mCoralIntakeMotor;

    // defining a variable motor speed
     private double mCoralIntakeMotorSpeed = 0.0;

    // configuration obj to configure the motor controller
    SparkFlexConfig config = new SparkFlexConfig();

    // initialize beam break sensor
    private DigitalInput mBeamBreakSensor;

    /* END CLASS VARIABLES */

    // constructor
    public CoralIntakeSubsystem() {
        mCoralIntakeMotor = new SparkMax(Constants.CORAL_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

        // what the motor should do when no input is given to the controller - in this case, brake
        config.idleMode(IdleMode.kBrake);

        mBeamBreakSensor = new DigitalInput(Constants.CORAL_BEAM_BREAK_DIO_CHANNEL);
    }

    /* MOTOR FUNCTIONS */

    // sets the intake motor speed
    public void setCoralIntakeSpeed(double pMotorSpeed) {
        mCoralIntakeMotorSpeed = pMotorSpeed;
    }

    // gets the intake motor speed
    public double getCoralIntakeSpeed() {
        return mCoralIntakeMotorSpeed;
    }

    public void stopCoralIntake() {
        mCoralIntakeMotor.set(0.0);
    }

    public void setCoralIntakeValue(double pMotorSpeed) {
        SmartDashboard.putNumber("Coral Intake: Motor Speed", pMotorSpeed);
        mCoralIntakeMotor.set(pMotorSpeed);
    }

    /* END MOTOR FUNCTIONS */


    /* BEAM BREAK SENSOR */
    public boolean hasCoral() {
        return !mBeamBreakSensor.get();
    }

    // figure out how +/- values correspond to intake/outtake
    public Command handleCoralCommand(DoubleSupplier pSource) {
        return run(() -> {
            setCoralIntakeSpeed(pSource.getAsDouble());
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run -> the "main" method for the coral intake
        setCoralIntakeValue(mCoralIntakeMotorSpeed);
    }
    
}
