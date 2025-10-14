package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneCoralTwoAlgaeCenter extends OrangeSequentialCommandGroup {

  public OneCoralTwoAlgaeCenter(Drive drive, Superstructure superstructure) {

    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
              PathPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        new ScoreCoral(superstructure, Level.L4, drive, true),
        new DescoreAlgae(superstructure, Level.L2, drive),
        AutoBuilder.followPath(Robot.GulfHotelToCenterBargeBackwards),
        new AlgaePrescoreAuto(superstructure, drive),
        new AlgaeScoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.CenterBargeBackwardsToLeave));
  }
}
