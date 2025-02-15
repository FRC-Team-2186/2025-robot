package frc.robot.commands.coral_intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {
    private final CoralIntakeSubsystem mCoralIntakeSubsystem;

    public CoralIntakeCommand(CoralIntakeSubsystem mCoralIntakeSubsystem) {
        this.mCoralIntakeSubsystem = mCoralIntakeSubsystem;
        addRequirements(mCoralIntakeSubsystem);
    }   

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mCoralIntakeSubsystem.setCoralIntakeSpeed(Constants.CORAL_INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            mCoralIntakeSubsystem.stopCoralIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return mCoralIntakeSubsystem.hasCoral();
    }
}
