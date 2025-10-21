package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.RigatoniIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;

public class ThreeRigatoniLeftPush extends OrangeSequentialCommandGroup {
  public ThreeRigatoniLeftPush(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_RIGATONI_LEFT_PUSH");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              PathPlannerPath path = Robot.ThreeRigatoniStartPushToJuliet;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeRigatoniStartPushToJuliet),
        new ScoreRigatoni(superstructure, Level.L4, drive, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.JulietToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreRigatoni(superstructure, Level.L4, drive, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.KiloToFeed)),
        AutoBuilder.followPath(Robot.FeedToLima),
        new ScoreRigatoni(superstructure, Level.L4, drive, false));
  }
}
