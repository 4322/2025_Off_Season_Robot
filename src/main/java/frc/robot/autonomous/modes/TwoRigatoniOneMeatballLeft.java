package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Buffet;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.MeatballPrescoreAuto;
import frc.robot.commands.auto.MeatballScoreAuto;
import frc.robot.commands.auto.RigatoniIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;

public class TwoRigatoniOneMeatballLeft extends OrangeSequentialCommandGroup {
  public TwoRigatoniOneMeatballLeft(
      DrivePan drivePan,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("TWO_RIGATONI_ONE_MEATBALL_LEFT");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              MealPlannerPath path = Robot.ThreeRigatoniStartToJuliet;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drivePan.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeRigatoniStartToJuliet),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.JulietToIndiaJuliet),
        new DescoreMeatball(superstructure, drivePan),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.MEATBALL_IDLE),
        AutoBuilder.followPath(Robot.IndiaJulietToLeftBarge),
        new MeatballPrescoreAuto(superstructure, drivePan),
        new WaitUntilCommand(
            () -> superstructure.spatulaAtSetpoint() && superstructure.layerCakeAtSetpoint()),
        AutoBuilder.followPath(Robot.LeftBargeToLeftMeatballScore),
        new MeatballScoreAuto(superstructure, drivePan),
        new WaitCommand(0.2),
        new WaitUntilCommand(() -> !superstructure.isMeatballHeld()),
        new Buffet(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.LeftMeatballScoreToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false));
  }
}
