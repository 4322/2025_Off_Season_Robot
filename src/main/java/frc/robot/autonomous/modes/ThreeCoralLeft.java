package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.AutoPoseReset;
import frc.robot.commands.CoralIntakeManual;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ThreeCoralLeft extends SequentialCommandGroup {
  public ThreeCoralLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_CORAL_LEFT");
    addRequirements(drive, superstructure, intakeSuperstructure);
    addCommands(
        new AutoPoseReset(
            drive, Robot.ThreeCoralStartToJuliet.getStartingHolonomicPose().get().getTranslation()),
        AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManual(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.JulietToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManual(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.KiloToFeed)),
        AutoBuilder.followPath(Robot.FeedToLima),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive));
  }
}
