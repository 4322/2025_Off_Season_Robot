package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import frc.robot.constants.Constants;

public class ElevatorIONitrate implements ElevatorIO {
  private final Nitrate leaderMotor;
  private final Nitrate followerMotor;

  // private final PIDPositionRequest elevatorPIDPositionRequest =
  // new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ElevatorIONitrate() {
    leaderMotor = new Nitrate(Constants.Elevator.leftMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.rightMotorID, MotorType.kCu60);

    // get position is an internal encoder, so we need to set it
    // 6 to 1 gear ratio for elevator first stage
    // 9 to 1 gear ratio for elevator second stage

  }

  @Override
  public void updateInputs(ElevatorIOInputs elevatorInputs) {
    // Implementation for updating inputs from the Nitrate hardware
  }

  @Override
  public void setElevatorEncoder() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }
}
