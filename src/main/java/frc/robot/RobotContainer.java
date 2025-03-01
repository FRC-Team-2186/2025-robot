// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
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

  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();

  private final SendableChooser<Command> mCommandChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
                                                                () -> mDriverController.getLeftY() * -1,
                                                                () -> mDriverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(mDriverController::getRightX)
                                                            .deadband(0.8)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", mCommandChooser);

    // Configure the trigger bindings
    configureBindings();

    mClimberSubsystem.setDefaultCommand(mClimberSubsystem.stopCommand());
    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(driveRobotOriented));

  }

  private void configureBindings() {

    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(driveRobotOriented));
    // var teleopDriveWithAngularVelocity =
    // SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
    // () -> mDriverController.getLeftX() * -1,
    // () -> mDriverController.getLeftY() * -1)
    // .withControllerRotationAxis(mDriverController::getRightX)
    // .deadband(0.5)
    // .scaleRotation(0.8)
    // .robotRelative(true);

    // mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(driveRobotOriented));

    mDriverController.y().whileTrue(mElevatorSubsystem.directCommand(() -> 0.25));
    mDriverController.a().whileTrue(mElevatorSubsystem.directCommand(() -> -0.25));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mCommandChooser.getSelected();
  }
}
