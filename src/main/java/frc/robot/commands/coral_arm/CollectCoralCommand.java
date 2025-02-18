package frc.robot.commands.coral_arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class CollectCoralCommand extends Command{
    private final CoralArmSubsystem mCoralArmSubsystem;

    public CollectCoralCommand(CoralArmSubsystem mCoralArmSubsystem) {
        this.mCoralArmSubsystem = mCoralArmSubsystem;
        addRequirements(mCoralArmSubsystem);
    }   

    @Override
    public void initialize() {
        mCoralArmSubsystem.usePidController();
    }

    @Override
    public void execute() {
        mCoralArmSubsystem.setDesiredAngle(null); // TODO determine angle of intake
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            mCoralArmSubsystem.stopCoralArmMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return mCoralArmSubsystem.atDesiredAngle();
    }
}
