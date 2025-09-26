// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveTwoMeters;
import frc.robot.commands.DrivetoReefAuto;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * P
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  @NotLogged
  private final CommandXboxController mDriverController = new CommandXboxController(0);

  @NotLogged
  private final CommandXboxController mOperatorController = new CommandXboxController(1);

  private final Vision mVision = new Vision();

  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem(mVision);
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final CoralIntakeSubsystem mIntakeSubsystem = new CoralIntakeSubsystem();

  private final SendableChooser<Command> mCommandChooser;

  SwerveInputStream mDriveFieldOriented = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
      () -> mDriverController.getLeftY() * -1,
      () -> -mDriverController.getLeftX())
      .withControllerRotationAxis(() -> -mDriverController.getRightX())
      .deadband(0.1)
      .scaleTranslation(0.8)

      .allianceRelativeControl(false);

  SwerveInputStream mDriveRobotOriented = mDriveFieldOriented.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream mSlowerDriveFieldOriented = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
      () -> mDriverController.getLeftY() * 0.25 * -1,
      () -> mDriverController.getLeftX() * 0.25)
      .withControllerRotationAxis(() -> -mDriverController.getRightX())
      .deadband(0.1)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mCommandChooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("elevator_to_l4", mElevatorSubsystem.moveToHeightCommand(Constants.L4_CORAL_INCHES));
    NamedCommands.registerCommand("elevator_to_l3", mElevatorSubsystem.moveToHeightCommand(Constants.L3_CORAL_INCHES));
    NamedCommands.registerCommand("elevator_to_l2", mElevatorSubsystem.moveToHeightCommand(Constants.L2_CORAL_INCHES));
    NamedCommands.registerCommand("elevator_to_bottom",
        mElevatorSubsystem.moveToHeightCommand(Constants.RESTING_CORAL_INCHES));
    NamedCommands.registerCommand("drop_coral", mIntakeSubsystem.ejectCoralCommand());
    NamedCommands.registerCommand("drivetrain_stop",
        mDrivetrainSubsystem.driveRobotOriented(() -> new ChassisSpeeds(0.0, 0.0, 0.0)));

    var driveToReefAuto = new DrivetoReefAuto(mDrivetrainSubsystem, 1, 1.7)
        .andThen(mElevatorSubsystem.moveToHeightCommand(Constants.L3_CORAL_INCHES))
        .andThen(mIntakeSubsystem.ejectCoralCommand());
    mCommandChooser.addOption("DriveToReef", driveToReefAuto);
    mCommandChooser.addOption("Drive2Meters",
        new DriveTwoMeters(mDrivetrainSubsystem).andThen(() -> mDrivetrainSubsystem.zeroGyroWithAlliance()));

    SmartDashboard.putData("Autonomous", mCommandChooser);

    mClimberSubsystem.setDefaultCommand(mClimberSubsystem.stopCommand());
    mElevatorSubsystem.setDefaultCommand(
        mElevatorSubsystem.directCommand(() -> -MathUtil.applyDeadband(mOperatorController.getRightY(), 0.1)));
    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(mDriveFieldOriented,
        () -> maxDrivetrainSpeedFactor(mElevatorSubsystem.getHeightPercent())));

    // On true
    mOperatorController.a().onTrue(mElevatorSubsystem.moveToHeightCommand(Constants.L2_CORAL_INCHES));
    mOperatorController.b().onTrue(mElevatorSubsystem.moveToHeightCommand(Constants.L3_CORAL_INCHES));
    mOperatorController.y().onTrue(mElevatorSubsystem.moveToHeightCommand(Constants.L4_CORAL_INCHES));
    mOperatorController.x().onTrue(mElevatorSubsystem.moveToHeightCommand(Constants.RESTING_CORAL_INCHES));

    // mOperatorController.leftBumper().and(mOperatorController.rightBumper()).whileTrue(mElevatorSubsystem.homeCommand());

    mDriverController.a().onTrue(new InstantCommand(() -> mDrivetrainSubsystem.zeroGyroWithAlliance()));

    mDriverController.leftTrigger().whileTrue(mDrivetrainSubsystem.driveFieldOriented(mSlowerDriveFieldOriented));
    mDriverController.leftBumper().whileTrue(mIntakeSubsystem.intakeCoralCommand());
    mDriverController.rightTrigger().whileTrue(mIntakeSubsystem.ejectCoralCommand());
    mDriverController.povDown().whileTrue(mClimberSubsystem.setStateCommand(Relay.Value.kForward));
    mDriverController.povUp().whileTrue(mClimberSubsystem.setStateCommand(Relay.Value.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mCommandChooser.getSelected();
  }

  public CommandXboxController getDriverController() {
    return mDriverController;
  }

  private double maxDrivetrainSpeedFactor(double pElevatorHeightPercent) {
    double percentClamped = Math.clamp(pElevatorHeightPercent, 0.0, 1.0);

    // Replace the exponent here to get different curves.
    // 1.0 = Linear
    // 2.0 = Quadratic
    return 1.0 - (1.0 - Constants.DRIVETRAIN_L4_SPEED_PERCENT) * Math.pow(percentClamped, 2.0);
  }
}
