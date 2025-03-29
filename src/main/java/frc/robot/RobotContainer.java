// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTwoMeters;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveInputStream;

import java.util.Collections;
import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.ImmutableAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**P
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

  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  // private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  // private final CoralArmSubsystem mCoralArmSubsystem = new CoralArmSubsystem();
  // private final CoralIntakeSubsystem mIntakeSubsystem = new CoralIntakeSubsystem();

  private final SendableChooser<Command> mCommandChooser;

  SwerveInputStream mDriveFieldOriented = SwerveInputStream.of(mDrivetrainSubsystem.getSwerveDrive(),
      () -> mDriverController.getLeftY() * -1,
      () -> mDriverController.getLeftX())
      .withControllerRotationAxis(mDriverController::getRightX)
      .deadband(0.1)
      .scaleTranslation(1.0)

      .allianceRelativeControl(true);

  SwerveInputStream mDriveRobotOriented = mDriveFieldOriented.copy().robotRelative(true)
      .allianceRelativeControl(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mCommandChooser = AutoBuilder.buildAutoChooser();

    // var awaitCoralCommand = Commands.sequence(
    //   mIntakeSubsystem.intakeCoralCommand().until(mIntakeSubsystem::hasCoral), mIntakeSubsystem.intakeCoralCommand().withTimeout(0.25));
    // var ejectCoralCommand = Commands.sequence(
    //   mIntakeSubsystem.ejectCoralCommand().until(() -> !mIntakeSubsystem.hasCoral()),
    //   mIntakeSubsystem.ejectCoralCommand().withTimeout(0.25));

    // var prepareForIntakeCommand = Commands.parallel(
    //   mCoralArmSubsystem.moveCoralToPositionCommand(Constants.CORAL_INTAKE_ANGLE),
    //   mElevatorSubsystem.moveToHeightCommand(Constants.ELEVATOR_INTAKE_POSITION));
    // var gotoRestingPositionCommand = Commands.parallel(
    //   mCoralArmSubsystem.moveCoralToPositionCommand(Constants.CORAL_RESTING_ANGLE_UP),
    //   mElevatorSubsystem.moveToHeightCommand(Constants.RESTING_CORAL_INCHES));
    // FIXME TEST THIS!!! 1 Note Coral Auto left
    var rightL1Coral = mDrivetrainSubsystem.driveRobotOriented(() -> new ChassisSpeeds(3, 4, 0)).withTimeout(3);
    var DrivetrainStop = mDrivetrainSubsystem.driveRobotOriented(() -> new ChassisSpeeds(0, 0, 0));
    // FIXME TEST THIS!!! 1 Note Coral Auto left
    var leftL1Coral = mDrivetrainSubsystem.driveRobotOriented(() -> new ChassisSpeeds(-3, 4, 0)).withTimeout(3);
    // FIXME TEST THIS!!! 1 Note Coral Auto middle
    var middleL1Coral = mDrivetrainSubsystem.driveRobotOriented(() -> new ChassisSpeeds(0, -5, 0)).withTimeout(5);
    // mCommandChooser.addOption("middleL1Coral", middleL1Coral);
    // mCommandChooser.addOption("leftL1Coral", leftL1Coral);
    // mCommandChooser.addOption("rightL1Coral", rightL1Coral);
    mCommandChooser.addOption("Drive2Meters", new DriveTwoMeters(mDrivetrainSubsystem).andThen(() -> mDrivetrainSubsystem.zeroGyroWithAlliance()));

    NamedCommands.registerCommand("DrivetrainStop", DrivetrainStop);
    // NamedCommands.registerCommand("elevator_to_l4", new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L4_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(Units.Degrees.of(-170))));
    // NamedCommands.registerCommand("drop_coral", ejectCoralCommand);
    // // NamedCommands.registerCommand("elevator_to_bottom", gotoRestingPositionCommand);
    // NamedCommands.registerCommand("await_coral", awaitCoralCommand);
    // NamedCommands.registerCommand("Zero Gyro", new InstantCommand(() -> mDrivetrainSubsystem.zeroGyroWithAlliance()));
    SmartDashboard.putData("Autonomous", mCommandChooser);

    Command driveFieldOrientedDirectAngle = mDrivetrainSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(mDriverController.getLeftY(), 0.1),
      () -> -MathUtil.applyDeadband(mDriverController.getLeftX(), 0.1),
      () -> mDriverController.getRightX());
      
    // mDrivetrainSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);

    // mClimberSubsystem.setDefaultCommand(mClimberSubsystem.stopCommand());
    // mElevatorSubsystem.setDefaultCommand(mElevatorSubsystem.directCommand(() -> 0.0));
    // mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveRobotOriented(mDriveRobotOriented));
    mDrivetrainSubsystem.setDefaultCommand(mDrivetrainSubsystem.driveFieldOriented(mDriveFieldOriented));
    // mIntakeSubsystem.setDefaultCommand(mIntakeSubsystem.handleCoralCommand(() -> 0.0));
    // mCoralArmSubsystem.setDefaultCommand(
    //   mCoralArmSubsystem.moveCoralArmCommand(() -> MathUtil.applyDeadband(mOperatorController.getRightY(), 0.15) * 0.25));

    // mDriverController.leftTrigger(0.25).whileTrue(ejectCoralCommand);
    // mDriverController.rightTrigger(0.25).whileTrue(awaitCoralCommand);

    // mOperatorController.a().whileTrue(mElevatorSubsystem.moveToHeightCommand(Constants.CORAL_HEIGHT_L2));
    // mOperatorController.b().whileTrue(mElevatorSubsystem.moveToHeightCommand(Constants.CORAL_HEIGHT_L3));
    // mOperatorController.y().whileTrue(mElevatorSubsystem.moveToHeightCommand(Constants.CORAL_HEIGHT_L4));
    // mOperatorController.x().whileTrue(mElevatorSubsystem.homeCommand());

    // mDriverController.a().whileTrue(mCoralArmSubsystem.moveCoralToPositionCommand(Units.Degrees.of(0.0)));
    // mDriverController.b().whileTrue(mCoralArmSubsystem.moveCoralToPositionCommand(Units.Degrees.of(45.0)));
    // mOperatorController.x().whileTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.RESTING_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(Constants.CORAL_RESTING_ANGLE_UP)));
    // mOperatorController.a().whileTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L2_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(Constants.L2_CORAL_ANGLE)));
    // mOperatorController.b().whileTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L3_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(Constants.L3_CORAL_ANGLE)));
    // mOperatorController.y().whileTrue(new ParallelCommandGroup(mElevatorSubsystem.moveToHeightCommand(Constants.L4_CORAL_INCHES), mCoralArmSubsystem.moveCoralToPositionCommand(Constants.L4_CORAL_ANGLE)));

    mDriverController.a().onTrue(new InstantCommand(() -> mDrivetrainSubsystem.zeroGyroWithAlliance()));
    // mOperatorController.rightStick().whileTrue(mElevatorSubsystem.directCommand(() -> mOperatorController.getRightY()));
    // mOperatorController.leftStick().whileTrue(mCoralArmSubsystem.moveCoralArmCommand(() -> mOperatorController.getLeftY()));

    
    // mDriverController.leftTrigger().whileTrue(mIntakeSubsystem.intakeCoralCommand());
    // mDriverController.rightTrigger().whileTrue(mIntakeSubsystem.ejectCoralCommand());
    mDriverController.povDown().whileTrue(mClimberSubsystem.setStateCommand(Relay.Value.kForward));
    mDriverController.povUp().whileTrue(mClimberSubsystem.setStateCommand(Relay.Value.kReverse));
    // Toggles between Resting Position(Approximately 89 degrees) and Intake Position(approximately 35 degrees)

    // var deferred = Commands.defer(() -> {
    //   return Commands.either(
    //     mCoralArmSubsystem.moveCoralToPositionCommand(Constants.CORAL_INTAKE_ANGLE),
    //     mCoralArmSubsystem.moveCoralToPositionCommand(Constants.CORAL_RESTING_ANGLE_UP),
    //     () -> mCoralArmSubsystem.getArmPosition().isNear(Constants.CORAL_RESTING_ANGLE_UP, 0.15));
    // }, Collections.singleton(mCoralArmSubsystem));

    // mDriverController.rightBumper().onTrue(Commands.either(
    //   prepareForIntakeCommand,
    //   gotoRestingPositionCommand,
    //   () -> mElevatorSubsystem.getPosition().isNear(Constants.ELEVATOR_INTAKE_POSITION, 0.05)));
    // mDriverController.leftBumper().onTrue(new ConditionalCommand(
    //   mElevatorSubsystem.moveToHeightCommand(Units.Inches.of(Constants.ELEVATOR_RESTING_POSITION_INCHES)),
    //   mElevatorSubsystem.moveToHeightCommand(Units.Inches.of(Constants.ELEVATOR_INTAKE_POSITION_INCHES)),
    //   mElevatorSubsystem::atIntakePosition));
    // Trigger toggleCoral = new Trigger(,() -> mElevatorSubsystem.atBottom());
    // mDriverController.rightBumper().toggleOnTrue(mElevatorSubsystem.moveToHeightCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new InstantCommand(() -> mDrivetrainSubsystem.driveFieldOriented())
    return new DriveTwoMeters(mDrivetrainSubsystem).andThen(() -> mDrivetrainSubsystem.zeroGyroWithAlliance());
    // return new WaitCommand(15);
    // return mCommandChooser.getSelected();
  }
  public CommandXboxController getDriverController(){
    return mDriverController;
  }
}
