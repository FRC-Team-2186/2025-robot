// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTwoMeters extends Command {
  DrivetrainSubsystem mSwerveDrive;
  Timer initial;
  /** Creates a new DriveTwoMeters. */
  /* To qualify for LEAVE points, a ROBOT must move such that its BUMPERS no longer overlap its ROBOT STARTING LINE at the end of AUTO.  */
  public DriveTwoMeters(DrivetrainSubsystem pSwerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerveDrive = pSwerveDrive;
    initial = new Timer();
    addRequirements(mSwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initial.start();
    mSwerveDrive.zeroGyro();
    System.out.println(initial);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mSwerveDrive.getSwerveDrive().drive(new ChassisSpeeds(0, 1, 0));
    // else {
    // mSwerveDrive.getSwerveDrive().drive(new ChassisSpeeds(-3, 0, 0));
    // }
    // System.out.println(initial + "time running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // !!!SID!!! - do we need to stop when the command ends or let it coast?
    mSwerveDrive.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("In is finished");
    if(initial.get() > 3){
      // System.out.println("Out of is finished command end");
      // System.out.println(initial.get());
      // System.out.println(Timer.getFPGATimestamp() - initial.get());
      // System.out.println(Timer.getFPGATimestamp());
      return true;
    }
    return false;
  }
}
