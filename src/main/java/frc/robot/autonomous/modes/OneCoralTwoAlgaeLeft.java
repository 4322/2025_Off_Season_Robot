package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneCoralTwoAlgaeLeft extends OrangeSequentialCommandGroup {

  public OneCoralTwoAlgaeLeft(Drive drive, Superstructure superstructure) {

    PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path = path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("ONE_CORAL_TWO_ALGAE_LEFT");
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
        new ScoreCoral(superstructure, Level.L4, drive, true, false),
        new DescoreAlgae(superstructure, drive),
        new ParallelCommandGroup(
            AutoBuilder.followPath(Robot.IJ_ToCenterAlgaeScore),
            new AlgaePrescoreAuto(superstructure, drive)),
        new AlgaeScoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.LeftAlgaeScoreBackwardsToKiloLima),
        new DescoreAlgae(superstructure, drive),
        AutoBuilder.followPath(Robot.KiloLimaToLeftBargeBackwards),
        new AlgaePrescoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.LeftBargeBackwardsToLeftAlgaeScoreBackwards),
        new AlgaeScoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.LeftAlgaeScoreBackwardsToLeave));
  }
}
