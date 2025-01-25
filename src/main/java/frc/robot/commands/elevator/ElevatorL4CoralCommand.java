package frc.robot.commands.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorL4CoralCommand extends Command{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final double desiredHeight = Constants.L4_CORAL_METERS;

    public ElevatorL4CoralCommand(ElevatorSubsystem mElevatorSubsystem) {
        this.mElevatorSubsystem = mElevatorSubsystem;
        addRequirements(mElevatorSubsystem);
    }   

    @Override
    public void initialize() {
        mElevatorSubsystem.usePidController();
        mElevatorSubsystem.resetRateLimiter();
    }

    @Override
    public void execute() {
        mElevatorSubsystem.setDesiredDistance(Units.Meters.of(desiredHeight));
    }

    @Override
    public void end(boolean interrupted) {
        mElevatorSubsystem.disablePidController();
    }

    @Override
    public boolean isFinished() {
        return mElevatorSubsystem.atDesiredDistance();
    }
}

