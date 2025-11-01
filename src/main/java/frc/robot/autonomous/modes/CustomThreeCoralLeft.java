package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.ScoreCoral;
import frc.robot.constants.FieldConstants.ReefFaceRobotHeading;
import frc.robot.constants.FieldConstants.ReefFaceTag;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector.EndEffectorStates;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import frc.robot.util.OrangeParallelCommandGroup;
import frc.robot.util.OrangeSequentialCommandGroup;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.AlgaeLevel;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;

public class CustomThreeCoralLeft extends OrangeSequentialCommandGroup {
  public CustomThreeCoralLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision,
      VisionObjectDetection visionObjectDetection) {

    ReefStatus reefCoral1 =
        new ReefStatus(
            false,
            false,
            new Rotation2d(
                Units.degreesToRadians(
                    Robot.alliance == Alliance.Blue
                        ? ReefFaceRobotHeading.IJ.blue
                        : ReefFaceRobotHeading.IJ.red)),
            ClosestReefPipe.RIGHT,
            L1Zone.MIDDLE,
            AlgaeLevel.L2,
            Robot.alliance == Alliance.Blue ? ReefFaceTag.IJ.idBlue : ReefFaceTag.IJ.idRed);

    ReefStatus reefCoral2 =
        new ReefStatus(
            false,
            false,
            new Rotation2d(
                Units.degreesToRadians(
                    Robot.alliance == Alliance.Blue
                        ? ReefFaceRobotHeading.KL.blue
                        : ReefFaceRobotHeading.KL.red)),
            ClosestReefPipe.LEFT,
            L1Zone.MIDDLE,
            AlgaeLevel.L2,
            Robot.alliance == Alliance.Blue ? ReefFaceTag.KL.idBlue : ReefFaceTag.KL.idRed);

    ReefStatus reefCoral3 =
        new ReefStatus(
            false,
            false,
            new Rotation2d(
                Units.degreesToRadians(
                    Robot.alliance == Alliance.Blue
                        ? ReefFaceRobotHeading.KL.blue
                        : ReefFaceRobotHeading.KL.red)),
            ClosestReefPipe.RIGHT,
            L1Zone.MIDDLE,
            AlgaeLevel.L2,
            Robot.alliance == Alliance.Blue ? ReefFaceTag.KL.idBlue : ReefFaceTag.KL.idRed);

    PathPlannerPath path = Robot.CustomThreeCoralStartToJuliet;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path = path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("CUSTOM_THREE_CORAL_LEFT");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              if (Robot.alliance == Alliance.Blue) {
                drive.resetPose(startPoseBlue);
              } else {
                drive.resetPose(startPoseRed);
              }
            }),
        new OrangeParallelCommandGroup(
            new ScoreCoral(superstructure, Level.L4, drive, false, true, reefCoral1),
            new OrangeSequentialCommandGroup(
                new WaitUntilCommand(
                    () ->
                        superstructure.getEndEffectorState()
                            == EndEffectorStates.RELEASE_CORAL_NORMAL),
                AutoBuilder.followPath(Robot.JulietToFeed1))),
        new OrangeParallelCommandGroup(
            AutoBuilder.followPath(Robot.JulietToFeed2),
            new CoralIntake(intakeSuperstructure, drive, visionObjectDetection)),
        new ScoreCoral(superstructure, Level.L4, drive, false, false, reefCoral2),
        new CoralIntake(intakeSuperstructure, drive, visionObjectDetection),
        new ScoreCoral(superstructure, Level.L4, drive, false, false, reefCoral3));
  }
}
