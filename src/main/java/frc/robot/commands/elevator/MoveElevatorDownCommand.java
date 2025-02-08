package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorDownCommand extends Command{

    private final ElevatorSubsystem mElevatorSubsystem;

    public MoveElevatorDownCommand(ElevatorSubsystem mElevatorSubsystem){
        this.mElevatorSubsystem = mElevatorSubsystem;
        addRequirements(mElevatorSubsystem);
    }

    @Override
    public void initialize() {
        mElevatorSubsystem.disablePidController();
    }

    @Override
    public void execute() {
        mElevatorSubsystem.setElevatorMotorValues(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        mElevatorSubsystem.disablePidController();
    }

    @Override
    public boolean isFinished() {
        return mElevatorSubsystem.atBottom();
    }
}
