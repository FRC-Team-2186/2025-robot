// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevator.MoveElevatorDownCommand;
import frc.robot.commands.elevator.MoveElevatorUpCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import swervelib.SwerveInputStream;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

  private final SendableChooser<Command> mCommandChooser;

  private final ElevatorSubsystem  mElevatorSubsystem = new ElevatorSubsystem();

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
  }

  private void configureBindings() {

    // var teleopDriveWithAngularVelocity = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
    //     () -> mDriverController.getLeftX() * -1,
    //     () -> mDriverController.getLeftY() * -1)
    //     .withControllerRotationAxis(mDriverController::getRightX)
    //     .deadband(0.5)
    //     .scaleRotation(0.8)
    //     .robotRelative(true);

    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(driveRobotOriented));

    mElevatorSubsystem.setDefaultCommand(mElevatorSubsystem.moveElevatorDirectCommand(() -> 0.0));
    mDriverController.y().whileTrue(mElevatorSubsystem.moveElevatorDirectCommand(() -> 0.25));
    mDriverController.a().whileTrue(mElevatorSubsystem.moveElevatorDirectCommand(() -> -0.25));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mCommandChooser.getSelected();
  }

  private DoubleSupplier negate(DoubleSupplier pSupplier) {
    return () -> pSupplier.getAsDouble() * -1;
  }
}
