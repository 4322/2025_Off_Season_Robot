package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;

public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private final Vision vision;
  private final DriveToPose driveToPose;
  private Timer L1OverrideTimer = new Timer();

  private enum ScoringState {
    DRIVE_BACK,
    SAFE_AREA,
    DRIVE_IN,
    DRIVE_IN_STOP,
  }

  ScoringState scoringState;

  
  public ScoreCoral(
      Superstructure superstructure,
      Superstructure.Level Level,
      Vision vision,
      DriveToPose driveToPose) {
    this.driveToPose = driveToPose;
    this.superstructure = superstructure;
    this.Level = Level;
    this.vision = vision;
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
    ReefStatus currentReefStatus = vision.getReefStatus();
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    ClosestReefPipe closestReefPipe = vision.getReefStatus().getClosestReefPipe();
    L1Zone closestL1Pipe = vision.getReefStatus().getClosestL1Zone();
    if ((Math.abs(Math.toDegrees(Math.atan2(y, x))) == -90
        || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -30
        || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 30
        || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 90
        || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 150
        || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -150)) {
      if (Level != Level.L1) {
        if (closestReefPipe == ClosestReefPipe.RIGHT) {
          closestReefPipe = ClosestReefPipe.LEFT;
        } else {
          closestReefPipe = ClosestReefPipe.RIGHT;
        }
      } else {
        L1OverrideTimer.start();
        if (L1OverrideTimer.hasElapsed(0.3)) {
          if (closestL1Pipe == L1Zone.RIGHT) {
            closestL1Pipe = L1Zone.MIDDLE;
            L1OverrideTimer.stop();
            L1OverrideTimer.reset();
          } else if (closestL1Pipe == L1Zone.LEFT) {
            closestL1Pipe = L1Zone.MIDDLE;
            L1OverrideTimer.stop();
            L1OverrideTimer.reset();
          } else {
            if (currentReefStatus.getClosestReefFaceAngle().getDegrees() == 0) {
              if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 0) {
                closestL1Pipe = L1Zone.RIGHT;
              } else if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 180) {
                closestL1Pipe = L1Zone.LEFT;
              }
            }
          }
        }
      }
    }

    switch (scoringState) {
      case DRIVE_BACK:
      new DriveToPose(null, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(vision.reefFace)));
        if (driveToPose.atGoal()) {
          scoringState = ScoringState.SAFE_AREA;
        }
        break;
    }

    //new DriveToPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(vision.reefFace)));


   
  


    if (RobotContainer.isScoringTriggerHeld() && !superstructure.isAutoOperationMode()) {
      superstructure.requestScoreCoral(Level);
    } else if (driveToPose.atGoal()) {
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
