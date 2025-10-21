package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Buffet;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.RigatoniIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

public class ThreeRigatoniRight extends OrangeSequentialCommandGroup {
  public ThreeRigatoniRight(
      DrivePan drivePan,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_RIGATONI_RIGHT");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              MealPlannerPath path = Robot.ThreeRigatoniStartToEcho;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drivePan.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeRigatoniStartToEcho),
        new InstantCommand(() -> Logger.recordOutput("Auto", "Finished path")),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new Buffet(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.EchoToFeed)),
        AutoBuilder.followPath(Robot.FeedToDelta),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new Buffet(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.DeltaToFeed)),
        AutoBuilder.followPath(Robot.FeedToCharlie),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false));
  }
}
