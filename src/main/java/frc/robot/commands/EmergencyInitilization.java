package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

// Never invoke this command while the arm is striaght up or on the back side of the robot!

public class EmergencyInitilization extends Command {
  private final IntakeSuperstructure intakeSuperstructure;
  private final Deployer deployer;
  private boolean deployerDone;
  private final Timer deployerStoppedTimer = new Timer();
  private boolean wasHomed;

  public EmergencyInitilization(
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Arm arm,
      Elevator elevator,
      Deployer deployer,
      Drive drive) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.deployer = deployer;
    // break out of everything
    addRequirements(superstructure, intakeSuperstructure, arm, elevator, deployer, drive);
  }

  @Override
  public void initialize() {
    // don't coast the arm because it could swing into the carbon fiber rod as the elevator descends
    wasHomed = (intakeSuperstructure.getState() != IntakeSuperstructure.IntakeSuperstates.HOMELESS);
    deployerDone = false;
    intakeSuperstructure.requestUnhome();
    deployerStoppedTimer.stop();
    deployerStoppedTimer.reset();
  }

  @Override
  public void execute() {
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
  }

  @Override
  public boolean isFinished() {
    return deployerDone;
  }

  @Override
  public void end(boolean interrupted) {
    if (wasHomed || !interrupted) {
      intakeSuperstructure.setReHome(); // don't be homeless
    }
  }
}
