// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */

  //Motors
  private  SparkFlex mCoralArmMotor;
  private SparkFlex mCoralIntakeMotor;

  private SparkFlex mAlgaeIntakeMotorRight;
  private SparkFlex mAlgaeIntakeMotorLeft;

  SparkAbsoluteEncoder coralArmEncoder;
  

  private final ArmFeedforward feedforward;




  public Intake() {

    mCoralArmMotor = new SparkFlex(19, MotorType.kBrushless);
    mCoralIntakeMotor = new SparkFlex(19, MotorType.kBrushless);
    
    mAlgaeIntakeMotorLeft = new SparkFlex(19, MotorType.kBrushless);
    mAlgaeIntakeMotorLeft = new SparkFlex(19, MotorType.kBrushless);
    
    coralArmEncoder = mCoralArmMotor.getAbsoluteEncoder();

    feedforward = new ArmFeedforward(0.0, 0.577, 0.0);

     

    

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  //Getter Values


}
