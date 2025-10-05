package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.driver;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.util.ReefStatus;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private final Superstructure superstructure;
  private Timer L1OverrideTimer = new Timer();
  private final Drive drive;
  private DriveToPose driveToPose;

  private Pose2d targetScoringPose;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();

  private Pose2d leftBranchScoringPos;
  private Pose2d rightBranchScoringPose;

  private Pose2d leftTroughScoringPose;
  private Pose2d middleTroughScoringPose;
  private Pose2d rightTroughScoringPose;

  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;

  public ScoreCoral(Superstructure superstructure, Superstructure.Level level, Drive drive) {
    this.superstructure = superstructure;
    this.level = level;
    this.drive = drive;
    driveToPose = new DriveToPose(drive, currentPoseRequest);
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
    superstructure.requestPrescoreCoral(
        level); // TODO: Move down to execute once automation is added to command

    // Fill in poses of all possible scoring locations for the level requested
    reefStatus = superstructure.getReefStatus();
    robotReefAngle = reefStatus.getClosestRobotAngle();

    if (level == Level.L1) {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        leftTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.leftTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        middleTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.middleTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      } else {
        leftTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.leftTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        middleTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.middleTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      }

      switch (reefStatus.getClosestL1Zone()) {
        case LEFT:
          targetScoringPose = leftTroughScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
        case MIDDLE:
          targetScoringPose = middleTroughScoringPose;
          superstructure.enableGlobalPose();
          break;
        case RIGHT:
          targetScoringPose = rightTroughScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
      }

    } else {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      } else {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      }

      switch (reefStatus.getClosestReefPipe()) {
        case LEFT:
          targetScoringPose = leftBranchScoringPos;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
        case RIGHT:
          targetScoringPose = rightBranchScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
      }
    }

    // Set current request to the safe pose
    currentPoseRequest =
        () ->
            targetScoringPose.transformBy(
                new Transform2d(
                    FieldConstants.KeypointPoses.reefSafeDistance
                        - Math.abs(targetScoringPose.getX()),
                    0,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)));
    ;
  }

  @Override
  public void execute() {
    // ****COMMENTS ON HOW TO WRITE COMMAND****

    if (superstructure.isAutoOperationMode()) {
      double x = -RobotContainer.driver.getLeftY();
      double y = -RobotContainer.driver.getLeftX();
      Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(y, x));
      double joystickMag = Math.hypot(x, y);
      if (Robot.alliance == DriverStation.Alliance.Red){
        joystickAngle = joystickAngle.rotateBy(Rotation2d.k180deg); //Convert joystick to feild relative
      }
      // Solve for the angle that the drive stick is at here
      joystickAngle =
      joystickAngle.rotateBy(robotReefAngle.unaryMinus());

      if (level == Level.L1) {
        
        if (joystickMag > 0.75) {
          if (joystickAngle.getDegrees() > 30) {
            targetScoringPose = leftTroughScoringPose;
            superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          } else if (joystickAngle.minus(robotReefAngle).getDegrees() < -30) {
            targetScoringPose = rightTroughScoringPose;
            superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          }
          else {
            targetScoringPose = middleTroughScoringPose;
            superstructure.enableGlobalPose();
          }
        }
        // Do the override stuff in here for l1
        // If override is detected, change the targetScoringPose variable and the single tag camera
        // to use
        // Use the range (robotReefAngle - 30 degrees, robotReefAngler + 30 degrees) to select the
        // middle, the rest make it left and right

        // The nice thing about using a supplier is that when you change the varable
        // "currentPoseRequest",
        // it will automatically forward to the drive to pose command and move the robot to that new
        // position
      } else {

        if (joystickMag > 0.75) {
          if (joystickAngle.getDegrees() > 0) {
            targetScoringPose = leftBranchScoringPos;
            superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          } else if (joystickAngle.getDegrees() < 0) {
            targetScoringPose = rightBranchScoringPose;
            superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          }
        }
        // Do the override stuff in here for other levels
        // If override is detected, change the targetScoringPose variable and the single tag camera
        // to use

        // The nice thing about using a supplier is that when you change the varable
        // "currentPoseRequest",
        // it will automatically forward to the drive to pose command and move the robot to that new
        // position
      }

      // Put your state machine here.
      // Cases:
      // Drive_To_Safe_Dist
      //    -Update the current pose request to the safe distance pose in case there was an override
      //    -Schedule the drive to pose command in this state going to the safe position
      //    -Raise arm and elevator while driving to safe position
      //    -Go to next state when arm and elevator at prescore setpoint
      // Drive_To_Score (Drives to scoring position once arm and elevator are fully up)
      //    -Just change the variable currentPoseRequest to the targetScoringPose
      //    -When drivebase at scoring position, automatically request score from superstructure
      //    -Go to next state when arm and elevator are at scoring setpoint
      // Drive_Back_Score
      //    -Drives back to safe zone and ends command when safe zone reached via the "running"
      // variable (look below for what "running" is)
      // Hold_Position
      //    -Cancel the drive to pose command
      //    -If at any moment in drive_to_score or drive_back_score we let go of the button, go to
      // this state
      //    -Only exit the state and go to drive_to_safe_dist when operation mode becomes manual or
      // drivebase is far enough away from reef
    } else {
      // Cancel the drive to pose command if it was scheduled
      // call superstructure.requestPrescoreCoral(level)
      // Have the isScoringTriggerHeld check here to request score when it's true

    }

    // Extra comments:

    // To end the command in auto mode, it's likely you'll need a variable called "running" which
    // will track when to exit the command
    // Either (we're in manaul operation mode and let go of the buttons), or (we're in auto
    // operation mode and running is false)
    // Ex: When we finish the drive back stuff and are a safe distance away, set running to false
    // and put it in the isFinished method so the command will end

    // Make sure to reset the state machine back to drive_to_safe_dist every time the command ends
    // or is reset via operation mode change

    // The safe distance pose will always be the line below. Try not to mind the messiness of the
    // math:
    // targetScoringPose.get().transformBy(new
    // Transform2d(FieldConstants.KeypointPoses.reefSafeDistance -
    // Math.abs(targetScoringPose.get().getX()), 0, robotReefAngle.rotateBy(Rotation2d.k180deg)));

    // ****END OF COMMENTS ON HOW TO WRITE COMMAND****

    // drive.requestAutoRotateMode(reefStatus.getClosestReefFaceAngle());

    // Above works
    // new DriveToPose(drive, new Pose2d(new Translation2d(),
    // reefStatus.getClosestReefFaceAngle()));
    // double x = -RobotContainer.driver.getLeftY();
    // double y = -RobotContainer.driver.getLeftX();
    // ClosestReefPipe closestReefPipe = superstructure.getReefStatus().getClosestReefPipe();
    // L1Zone closestL1Pipe = superstructure.getReefStatus().getClosestL1Zone();

    // if ((Math.abs(Math.toDegrees(Math.atan2(y, x))) == -90
    //     || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -30
    //     || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 30
    //     || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 90
    //     || Math.abs(Math.toDegrees(Math.atan2(y, x))) == 150
    //     || Math.abs(Math.toDegrees(Math.atan2(y, x))) == -150)) {
    //   if (Level != Level.L1) {
    //     if (closestReefPipe == ClosestReefPipe.RIGHT) {
    //       closestReefPipe = ClosestReefPipe.LEFT;
    //     } else {
    //       closestReefPipe = ClosestReefPipe.RIGHT;
    //     }
    //   } else {
    //     L1OverrideTimer.start();
    //     if (L1OverrideTimer.hasElapsed(0.3)) {
    //       if (closestL1Pipe == L1Zone.RIGHT) {
    //         closestL1Pipe = L1Zone.MIDDLE;
    //         L1OverrideTimer.stop();
    //         L1OverrideTimer.reset();
    //       } else if (closestL1Pipe == L1Zone.LEFT) {
    //         closestL1Pipe = L1Zone.MIDDLE;
    //         L1OverrideTimer.stop();
    //         L1OverrideTimer.reset();
    //       } else {
    //         if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 0) {
    //           closestL1Pipe = L1Zone.RIGHT;
    //         } else if (Math.abs(Math.toDegrees(Math.atan2(y, x))) == 180) {
    //           closestL1Pipe = L1Zone.LEFT;
    //         }
    //       }
    //     }
    //   }
    // }

    // if (closestL1Pipe == L1Zone.RIGHT) {
    // new DriveToPose(drive, new Pose2d(new Translation2d(),
    // reefStatus.getClosestReefFaceAngle()));
    // } else if (closestL1Pipe == L1Zone.LEFT) {

    // } else {

    // }

    if (RobotContainer.isScoringTriggerHeld()) {
      superstructure.requestScoreCoral(level);
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
    superstructure.enableGlobalPose();

    if (driveToPose.isScheduled()) {
      driveToPose.cancel();
    }
  }
}
