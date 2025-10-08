package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.CoralIntakeManual;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ThreeCoralRight extends SequentialCommandGroup {
  public ThreeCoralRight(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_CORAL_RIGHT");
    addRequirements(drive, superstructure, intakeSuperstructure);
    addCommands(
        new InstantCommand(() -> drive.resetPose(Robot.ThreeCoralStartToEcho.getStartingHolonomicPose().get())),
        AutoBuilder.followPath(Robot.ThreeCoralStartToEcho),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManual(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.EchoToFeed)),
        AutoBuilder.followPath(Robot.FeedToDelta),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManual(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.DeltaToFeed)),
        AutoBuilder.followPath(Robot.FeedToCharlie),
        new ScoreCoral(superstructure, Superstructure.Level.L4, drive));
  }
}
