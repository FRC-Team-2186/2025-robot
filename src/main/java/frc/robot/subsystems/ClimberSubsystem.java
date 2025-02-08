package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
  private final Relay mRelay = new Relay(Constants.CLIMBER_RELAY_PORT_ID);
  private Relay.Value mRelayState = Relay.Value.kOff;

  public ClimberSubsystem() {
  }

  public Relay.Value getState() {
    return mRelayState;
  }

  public Command setStateCommand(Relay.Value pNewState) {
    return run(() -> {
      mRelayState = pNewState;
    });
  }

  public Command stopCommand() {
    return setStateCommand(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    mRelay.set(mRelayState);
  }
}
