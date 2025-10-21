package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Buffet;
import frc.robot.Robot;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.MeatballPrescoreAuto;
import frc.robot.commands.auto.MeatballScoreAuto;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneRigatoniOneMeatballCenter extends OrangeSequentialCommandGroup {
  public OneRigatoniOneMeatballCenter(DrivePan drivePan, Superstructure superstructure) {
    setName("ONE_RIGATONI_ONE_MEATBALL_CENTER");
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
        new ScoreRigatoni(superstructure, Level.L4, drivePan, true),
        new DescoreMeatball(superstructure, drivePan),
        new Buffet(
            AutoBuilder.followPath(Robot.GulfHotelToCenterBargeBackwards),
            new MeatballPrescoreAuto(superstructure, drivePan)),
        new MeatballScoreAuto(superstructure, drivePan),
        AutoBuilder.followPath(Robot.CenterMeatballScoreBackwardsToLeave));
  }
}
