package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class CoralIntakeSubsystem extends SubsystemBase {

  /** SETTING UP CLASS VARIABLES */

  // intake motor is a Neo 550
  private final SparkMax mCoralIntakeMotor = new SparkMax(Constants.CORAL_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

  // initialize beam break sensor
  @Logged
  private final DigitalInput mBeamBreakSensor = new DigitalInput(Constants.CORAL_BEAM_BREAK_DIO_CHANNEL);

  /* END CLASS VARIABLES */

  // constructor
  public CoralIntakeSubsystem() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    mCoralIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /* BEAM BREAK SENSOR */
  public boolean hasCoral() {
    return !mBeamBreakSensor.get();
  }

  // figure out how +/- values correspond to intake/outtake
  public Command handleCoralCommand(DoubleSupplier pSource) {
    return run(() -> {
      mCoralIntakeMotor.set(pSource.getAsDouble());
    });
  }

  public Command ejectCoralCommand() {
    return run(() -> mCoralIntakeMotor.set(0.5)) //
        .withTimeout(0.25) //
        .finallyDo(() -> mCoralIntakeMotor.set(0.0));
  }

  public Command intakeCoralCommand() {
    return run(() -> mCoralIntakeMotor.set(-0.5)).finallyDo(() -> mCoralIntakeMotor.set(0.0));
  }
}
