package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.CoralIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

public class ThreeCoralLeft extends OrangeSequentialCommandGroup {
  public ThreeCoralLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_CORAL_LEFT");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
              PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet),
        new InstantCommand(() -> Logger.recordOutput("Auto", "Finished path")),
        new ScoreCoral(superstructure, Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.JulietToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreCoral(superstructure, Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.KiloToFeed)),
        AutoBuilder.followPath(Robot.FeedToLima),
        new ScoreCoral(superstructure, Level.L4, drive));
  }
}
