package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSuperstructure extends SubsystemBase {

  private boolean requestRetractIdle;
  private boolean requestFeed;
  private boolean requestReject;
  private boolean requestIntakeEject;

  private IntakeSuperstates state = IntakeSuperstates.START;

  public static enum IntakeSuperstates {
    START,
    RETRACT_IDLE,
    FEED,
    REJECT,
    INTAKE_EJECT
  }

  public IntakeSuperstructure() {}

  @Override
  public void periodic() {
    switch (state) {
      case START:
        break;
      case RETRACT_IDLE:
        break;
      case FEED:
        break;
      case REJECT:
        break;
      case INTAKE_EJECT:
        break;
    }
  }

  public void requestRetractIdle() {}

  public void requestIntake() {
    // Transitions to Feeding or Rejecting based on if coral in robot
  }

  public void requestEject() {}

  public boolean isCoralDetectedPickupArea() {
    return true; // TODO indexer.isCoralDetectedPickupArea()
  }

  public boolean isCoralDetectedIndexer() {
    return true; // TODO indexer.isCoralDetectedIndexer()
  }
}
