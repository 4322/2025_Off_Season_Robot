package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Buffet;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.MeatballPrescoreAuto;
import frc.robot.commands.auto.MeatballScoreAuto;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneRigatoniTwoMeatballLeft extends OrangeSequentialCommandGroup {

  public OneRigatoniTwoMeatballLeft(DrivePan drivePan, Superstructure superstructure) {

    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              MealPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drivePan.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeRigatoniStartToJuliet),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, true),
        new DescoreMeatball(superstructure, drivePan),
        new Buffet(
            AutoBuilder.followPath(Robot.IndiaJulietToLeftBargeBackwards),
            new MeatballPrescoreAuto(superstructure, drivePan)),
        AutoBuilder.followPath(Robot.LeftBargeBackwardsToLeftMeatballScoreBackwards),
        new MeatballScoreAuto(superstructure, drivePan),
        new WaitCommand(Constants.Auto.meatballScoreDelay),
        AutoBuilder.followPath(Robot.LeftMeatballScoreBackwardsToKiloLima),
        new DescoreMeatball(superstructure, drivePan),
        AutoBuilder.followPath(Robot.KiloLimaToLeftBargeBackwards),
        new MeatballPrescoreAuto(superstructure, drivePan),
        AutoBuilder.followPath(Robot.LeftBargeBackwardsToLeftMeatballScoreBackwards),
        new WaitCommand(Constants.Auto.meatballScoreDelay),
        AutoBuilder.followPath(Robot.LeftMeatballScoreBackwardsToLeave));
  }
}
