// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */

  //Motors
  private  SparkFlex mCoralArmMotor;
  private SparkFlex mCoralIntakeMotor;

  private SparkFlex mAlgaeIntakeMotorRight;
  private SparkFlex mAlgaeIntakeMotorLeft;

  SparkAbsoluteEncoder coralArmEncoder;
  
  //Arm Pid
  private final ArmFeedforward feedforward;
  private final ProfiledPIDController armPIDController;




  public Intake() {

    mCoralArmMotor = new SparkFlex(19, MotorType.kBrushless);
    mCoralIntakeMotor = new SparkFlex(19, MotorType.kBrushless);
    
    mAlgaeIntakeMotorLeft = new SparkFlex(19, MotorType.kBrushless);
    mAlgaeIntakeMotorLeft = new SparkFlex(19, MotorType.kBrushless);
    
    coralArmEncoder = mCoralArmMotor.getAbsoluteEncoder();

    feedforward = new ArmFeedforward(0.0, 0.577, 0.0);
    armPIDController = new ProfiledPIDController(.55, 0, 0, null);
    armPIDController.setGoal(0);
     

    

  }



  //Intakes
  public void setCoralIntakeVoltage(double voltage) {
    mCoralArmMotor.setVoltage(voltage);

  }

  public void setAlgaeIntakeVoltage(double voltage) {
    mAlgaeIntakeMotorLeft.setVoltage(voltage);
    mAlgaeIntakeMotorRight.setVoltage(-voltage);
  }

  public double getCoralIntakeCurrent() {
    return mCoralIntakeMotor.getAppliedOutput();
  }
  public double getAlgaeIntakeCurrent() {
    return mAlgaeIntakeMotorLeft.getAppliedOutput();
  }


  //Coral Arm
    public double getMeasurement() {
    return coralArmEncoder.getPosition() * 360;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    //PID Algorithm
    double position = getMeasurement();
    double velocity = armPIDController.getSetpoint().velocity;
   
    double armPIDControllerOutput = armPIDController.calculate(10);
    
     double feedforwardOutput = feedforward.calculate(position, velocity);
    double output = armPIDControllerOutput + feedforwardOutput;

    output = MathUtil.clamp(output, -0.05, 0.05);

    mCoralArmMotor.set(output);
  }


  //Getter Values


}
