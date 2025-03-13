// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.ImmutableAngle;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
  private final CommandXboxController mSecondController  = new CommandXboxController(1);

  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final CoralArmSubsystem mCoralArmSubsystem = new CoralArmSubsystem();
  // private final CoralArmSubsystem mArmSubsystem = new CoralArmSubsystem();
  // private final CoralIntakeSubsystem mIntakeSubsystem = new
  // CoralIntakeSubsystem();

  private final SendableChooser<Command> mCommandChooser;

  SwerveInputStream mDriveAngularVelocity = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
      () -> mDriverController.getLeftY() * -1,
      () -> mDriverController.getLeftX() * -1)
      .withControllerRotationAxis(mDriverController::getRightX)
      .deadband(0.8)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream mDriveFieldOriented = mDriveAngularVelocity.copy().robotRelative(true)
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
    mElevatorSubsystem.setDefaultCommand(mElevatorSubsystem.directCommand(() -> 0.0));
    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(mDriveFieldOriented));
  }

  private void configureBindings() {
    mSecondController.x().onTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.RESTING_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(new ImmutableAngle(35, 0.610865, Units.Degrees))));
    mSecondController.a().onTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L2_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(new ImmutableAngle(35, 0.610865, Units.Degrees))));
    mSecondController.b().onTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L3_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(new ImmutableAngle(35, 0.610865, Units.Degrees))));
    mSecondController.y().onTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L4_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(new ImmutableAngle(170, 2.96706, Units.Degrees))));


    mDriverController.rightTrigger().whileTrue(mCoralArmSubsystem.setCoralIntakeSpeedCommand(() -> mSecondController.getRightTriggerAxis()));
    mDriverController.leftTrigger().whileTrue(mCoralArmSubsystem.setCoralIntakeSpeedCommand(() -> mSecondController.getRightTriggerAxis() * -1));
    mDriverController.povUp().onTrue(mClimberSubsystem.setStateCommand(Relay.Value.kForward));
    mDriverController.povDown().onTrue(mClimberSubsystem.setStateCommand(Relay.Value.kReverse));

    // Trigger toggleCoral = new Trigger(,() -> mElevatorSubsystem.atBottom());
    // FIXME Should toggle between elevator positions only if elevator is at resting position
    // mDriverController.rightBumper().toggleOnTrue(mElevatorSubsystem.moveToHeightCommand());
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
