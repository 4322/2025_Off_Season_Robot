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
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
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

public class TwoCoralOneAlgaeLeft extends OrangeSequentialCommandGroup {
  public TwoCoralOneAlgaeLeft(
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

    PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path = path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("TWO_CORAL_ONE_ALGAE_LEFT");
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
        new ScoreCoral(superstructure, Level.L4, drive, false, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.JulietToIndiaJuliet),
        new DescoreAlgae(superstructure, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.ALGAE_IDLE),
        AutoBuilder.followPath(Robot.IJ_ToCenterAlgaeScore),
        new AlgaePrescoreAuto(superstructure, drive),
        new WaitUntilCommand(
            () -> superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()),
        AutoBuilder.followPath(Robot.LeftBargeToLeftAlgaeScore),
        new AlgaeScoreAuto(superstructure, drive),
        new WaitUntilCommand(() -> !superstructure.isAlgaeHeld()),
        new ParallelCommandGroup(
            // new CoralIntakeManualAuto(intakeSuperstructure, drive, vision, false),
            AutoBuilder.followPath(Robot.LeftAlgaeScoreToFeed)),
        new ScoreCoral(superstructure, Level.L4, drive, false, false, reefCoral2));
  }
}
