package frc.robot.commands;

import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;




public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private final ReefStatus reefStatus;
  private final Vision vision;
  private Timer L1OverrideTimer = new Timer();

  public ScoreCoral(Superstructure superstructure, Superstructure.Level Level, Vision vision, ReefStatus reefStatus) {
    this.superstructure = superstructure;
    this.Level = Level;
    this.vision = vision;
    this.reefStatus = reefStatus;
    addRequirements(superstructure);
  }
  @Override
  public void initialize() {
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
    superstructure.requestPrescoreCoral(Level);
  }

  @Override
  public void execute() {
double x = -RobotContainer.driver.getLeftY();
double y = -RobotContainer.driver.getLeftX();
ClosestReefPipe closestReefPipe = reefStatus.getClosestReefPipe(); 
L1Zone closestL1Pipe = reefStatus.getClosestL1Zone(); 
if ((Math.abs(Math.toDegrees(Math.atan2(y, x))) == -90 || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -30 || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 30 || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 90 || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 150 || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -150) ) {
  if (Level != Level.L1){
  if (closestReefPipe == ClosestReefPipe.RIGHT){
    closestReefPipe = ClosestReefPipe.LEFT;
  }
  else {
    closestReefPipe = ClosestReefPipe.RIGHT;
  }
}
else {
  L1OverrideTimer.start();
  if (L1OverrideTimer.hasElapsed(0.3)){
    if (closestL1Pipe == L1Zone.RIGHT){
      closestL1Pipe = L1Zone.MIDDLE;
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
    }
    else if (closestL1Pipe == L1Zone.LEFT){
      closestL1Pipe = L1Zone.MIDDLE;
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
    }
    else {
    if(vision.reefFace == 0){
      if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 0){
        closestL1Pipe = L1Zone.RIGHT;
      }
      else if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 180){
        closestL1Pipe = L1Zone.LEFT;
      }
    }
    }
    
  }
}
}


    if (RobotContainer.isScoringTriggerHeld()) {
      superstructure.requestScoreCoral(Level);
    }
  }

  @Override
  public boolean isFinished() {
    return !driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
