package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.spatula.Spatula;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.layerCake.LayerCake;

// Never invoke this command while the spatula is striaght up or on the back side of the robot!

public class EmergencyInitilization extends Command {
  private final Superstructure superstructure;
  private final IntakeSuperstructure intakeSuperstructure;
  private final Spatula spatula;
  private final LayerCake layerCake;
  private final Deployer deployer;
  private final DrivePan drivePan;
  private boolean spatulaDone;
  private boolean layerCakeDone;
  private boolean deployerDone;
  private final Timer spatulaStoppedTimer = new Timer();
  private final Timer layerCakeStoppedTimer = new Timer();
  private final Timer deployerStoppedTimer = new Timer();
  private boolean wasHomed;

  public EmergencyInitilization(
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Spatula spatula,
      LayerCake layerCake,
      Deployer deployer,
      DrivePan drivePan) {
    this.superstructure = superstructure;
    this.intakeSuperstructure = intakeSuperstructure;
    this.spatula = spatula;
    this.layerCake = layerCake;
    this.deployer = deployer;
    this.drivePan = drivePan;
    // break out of everything
    addRequirements(superstructure, intakeSuperstructure, spatula, layerCake, deployer, drivePan);
  }

  @Override
  public void initialize() {
    // don't coast the spatula because it could swing into the carbon fiber rod as the layerCake descends
    wasHomed = (superstructure.getState() != Superstates.HOMELESS);
    spatulaDone = false;
    layerCakeDone = false;
    deployerDone = false;
    superstructure.requestUnhome();
    intakeSuperstructure.requestUnhome();
    layerCake.emergencyHoming();
    layerCakeStoppedTimer.stop();
    layerCakeStoppedTimer.reset();
    deployerStoppedTimer.stop();
    deployerStoppedTimer.reset();
    spatulaStoppedTimer.stop();
    spatulaStoppedTimer.reset();
  }

  @Override
  public void execute() {
    if (!layerCakeDone) {
      if (Math.abs(layerCake.emergencyHoming()) <= Constants.LayerCake.initializationCompleteSpeed) {
        layerCakeStoppedTimer.start();
      } else {
        layerCakeStoppedTimer.stop();
        layerCakeStoppedTimer.reset();
      }
      if (layerCakeStoppedTimer.hasElapsed(Constants.LayerCake.initializationCompleteSec)) {
        layerCake.setEmergencyHomingComplete();
        layerCakeDone = true;
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
    if (!spatulaDone && layerCakeDone && deployerDone && superstructure.layerCakeAtSetpoint()) {
      if (Math.abs(spatula.emergencyHoming()) <= Constants.Spatula.initializationCompleteSpeed) {
        spatulaStoppedTimer.start();
      } else {
        spatulaStoppedTimer.stop();
        spatulaStoppedTimer.reset();
      }
      if (spatulaStoppedTimer.hasElapsed(Constants.Spatula.initializationCompleteSec)) {
        spatula.setEmergencyHomingComplete();
        spatulaDone = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return spatulaDone && layerCakeDone && deployerDone;
  }

  @Override
  public void end(boolean interrupted) {
    if (wasHomed || !interrupted) {
      superstructure.requestReHome(); // don't be homeless, even if spatula/layerCake not yet done
    }
  }
}
