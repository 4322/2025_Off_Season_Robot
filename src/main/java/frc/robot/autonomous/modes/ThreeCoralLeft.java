package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.CoralIntakeManualAuto;
import frc.robot.constants.FieldConstants.ReefFaceRobotHeading;
import frc.robot.constants.FieldConstants.ReefFaceTag;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.AlgaeLevel;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;
import org.littletonrobotics.junction.Logger;

public class ThreeCoralLeft extends OrangeSequentialCommandGroup {
  public ThreeCoralLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {

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

    PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path = path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("THREE_CORAL_LEFT");
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
        AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet),
        new InstantCommand(() -> Logger.recordOutput("Auto", "Finished path")),
        new ScoreCoral(superstructure, Level.L4, drive, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.JulietToFeed)),
        new ScoreCoral(superstructure, Level.L4, drive, false, reefCoral2),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.KiloToFeed)),
        new ScoreCoral(superstructure, Level.L4, drive, false, reefCoral3));
  }
}
