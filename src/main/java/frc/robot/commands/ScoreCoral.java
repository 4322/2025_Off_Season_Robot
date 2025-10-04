package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;

public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private Timer L1OverrideTimer = new Timer();
  private final Vision vision;
  private final Drive drive;

  public ScoreCoral(
      Superstructure superstructure, Superstructure.Level Level, Drive drive, Vision vision) {
    this.superstructure = superstructure;
    this.Level = Level;
    this.drive = drive;
    this.vision = vision;
    addRequirements(superstructure, drive, vision);
  }

  @Override
  public void initialize() {
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
  }

  @Override
  public void execute() {
    ReefStatus reefStatus = vision.getReefStatus();
    new DriveToPose(drive, new Pose2d(new Translation2d(), reefStatus.getClosestReefFaceAngle()));
    superstructure.requestPrescoreCoral(Level);
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
            if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 0) {
              closestL1Pipe = L1Zone.RIGHT;
            } else if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 180) {
              closestL1Pipe = L1Zone.LEFT;
            }
          }
        }
      }
    }

    // if (closestL1Pipe == L1Zone.RIGHT) {
    // new DriveToPose(drive, new Pose2d(new Translation2d(),
    // reefStatus.getClosestReefFaceAngle()));
    // } else if (closestL1Pipe == L1Zone.LEFT) {

    // } else {

    // }

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
