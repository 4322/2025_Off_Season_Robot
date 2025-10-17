package frc.robot.commands;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.elevator.Elevator;

// Never invoke this command while the arm is striaght up or on the back side of the robot!

public class EmergencyInitilization extends Command {
  private final Superstructure superstructure;
  private final IntakeSuperstructure intakeSuperstructure;
  private final Arm arm;
  private final Elevator elevator;
  private final Deployer deployer;
  private boolean armDone;
  private boolean elevatorDone;
  private boolean deployerDone;
  private final Timer armStoppedTimer = new Timer();
  private final Timer elevatorStoppedTimer = new Timer();
  private final Timer deployerStoppedTimer = new Timer();

  public EmergencyInitilization(
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Arm arm,
      Elevator elevator,
      Deployer deployer) {
    this.superstructure = superstructure;
    this.intakeSuperstructure = intakeSuperstructure;
    this.arm = arm;
    this.elevator = elevator;
    this.deployer = deployer;
    addRequirements(superstructure, arm, elevator, deployer);
  }

  @Override
  public void initialize() {
    // can't unhome the superstructure because it only supports re-homing in the standard positions
    armDone = false;
    elevatorDone = false;
    deployerDone = false;
    arm.stop(IdleMode.kCoast);
    intakeSuperstructure.requestUnhome();
    elevator.emergencyHoming();
    elevatorStoppedTimer.stop();
    elevatorStoppedTimer.reset();
    deployerStoppedTimer.stop();
    deployerStoppedTimer.reset();
    armStoppedTimer.stop();
    armStoppedTimer.reset();
  }

  @Override
  public void execute() {
    if (!elevatorDone) {
      if (elevator.emergencyHoming() <= Constants.Elevator.initializationCompleteSpeed) {
        elevatorStoppedTimer.start();
      } else {
        elevatorStoppedTimer.stop();
        elevatorStoppedTimer.reset();
      }
      if (elevatorStoppedTimer.hasElapsed(Constants.Elevator.initializationCompleteSec)) {
        elevator.setEmergencyHomingComplete();
        elevatorDone = true;
      }
    }
    if (!deployerDone) {
      if (deployer.emergencyHoming() <= Constants.Deployer.initializationCompleteSpeed) {
        deployerStoppedTimer.start();
      } else {
        deployerStoppedTimer.stop();
        deployerStoppedTimer.reset();
      }
      if (deployerStoppedTimer.hasElapsed(Constants.Deployer.initializationCompleteSec)) {
        intakeSuperstructure.setHome();
        deployerDone = true;
      }
    }
    if (!armDone && elevatorDone && deployerDone) {
      arm.stop(IdleMode.kBrake);
      if (arm.emergencyHoming() <= Constants.Arm.initializationCompleteSpeed) {
        armStoppedTimer.start();
      } else {
        armStoppedTimer.stop();
        armStoppedTimer.reset();
      }
      if (armStoppedTimer.hasElapsed(Constants.Arm.initializationCompleteSec)) {
        arm.setEmergencyHomingComplete();
        armDone = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return armDone && elevatorDone && deployerDone;
  }

  @Override
  public void end(boolean interrupted) {}
}
