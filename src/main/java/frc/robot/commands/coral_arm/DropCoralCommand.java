package frc.robot.commands.coral_arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class DropCoralCommand extends Command {

    private final CoralArmSubsystem mCoralArmSubsystem;
    private final String reefLevel;

    public DropCoralCommand(CoralArmSubsystem mCoralArmSubsystem, String reefLevel) {
        this.mCoralArmSubsystem = mCoralArmSubsystem;
        this.reefLevel = reefLevel;
        addRequirements(mCoralArmSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (reefLevel) {
            case "L2":
                mCoralArmSubsystem.setDesiredAngle(null); // TODO determine angle of drop
            case "L3":
                mCoralArmSubsystem.setDesiredAngle(null); // TODO determine angle of drop
            case "L4":
                mCoralArmSubsystem.setDesiredAngle(null); // TODO determine angle of drop
            default:
                mCoralArmSubsystem.setDesiredAngle(mCoralArmSubsystem.getArmPosition()); // do nothing
        }
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
