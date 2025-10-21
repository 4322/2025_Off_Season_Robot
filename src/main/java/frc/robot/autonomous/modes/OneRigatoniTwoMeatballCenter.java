package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.MeatballPrescoreAuto;
import frc.robot.commands.auto.MeatballScoreAuto;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneRigatoniTwoMeatballCenter extends OrangeSequentialCommandGroup {

  public OneRigatoniTwoMeatballCenter(Drive drive, Superstructure superstructure) {
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              PathPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        new ScoreRigatoni(superstructure, Level.L4, drive, true),
        new DescoreMeatball(superstructure, drive),
        new ParallelCommandGroup(
            AutoBuilder.followPath(Robot.GulfHotelToCenterBargeBackwards),
            new MeatballPrescoreAuto(superstructure, drive)),
        new MeatballScoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.CenterMeatballScoreBackwardsToIndiaJuliet),
        new DescoreMeatball(superstructure, drive),
        new ParallelCommandGroup(
            AutoBuilder.followPath(Robot.IndiaJulietToCenterBargeBackwards),
            new MeatballPrescoreAuto(superstructure, drive)),
        new MeatballScoreAuto(superstructure, drive),
        AutoBuilder.followPath(Robot.CenterBargeBackwardsToLeave));
  }
}
