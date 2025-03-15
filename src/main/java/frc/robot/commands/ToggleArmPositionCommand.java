package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class ToggleArmPositionCommand extends Command {
    private final CoralArmSubsystem mArmSubsystem;
    private final Angle mAngleA;
    private final Angle mAngleB;

    public ToggleArmPositionCommand(Angle pAngleA, Angle pAngleB, CoralArmSubsystem pArmSubsystem) {
        mArmSubsystem = pArmSubsystem;
        mAngleA = pAngleA;
        mAngleB = pAngleB;

        addRequirements(pArmSubsystem);
    }

    
}
