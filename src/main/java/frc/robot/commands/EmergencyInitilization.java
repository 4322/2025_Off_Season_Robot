package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
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
  private boolean wasHomed;

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
    // don't coast the arm because it could swing into the carbon fiber rod as the elevator descends
    wasHomed = (superstructure.getState() != Superstates.HOMELESS);
    armDone = false;
    elevatorDone = false;
    deployerDone = false;
    superstructure.requestUnhome();
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
      if (Math.abs(elevator.emergencyHoming()) <= Constants.Elevator.initializationCompleteSpeed) {
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
      if (Math.abs(deployer.emergencyHoming()) <= Constants.Deployer.initializationCompleteSpeed) {
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
    if (!armDone && elevatorDone && deployerDone && superstructure.elevatorAtSetpoint()) {
      if (Math.abs(arm.emergencyHoming()) <= Constants.Arm.initializationCompleteSpeed) {
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
  public void end(boolean interrupted) {
    if (wasHomed || !interrupted) {
      superstructure.requestReHome(); // don't be homeless, even if arm/elevator not yet done
    }
  }
}
