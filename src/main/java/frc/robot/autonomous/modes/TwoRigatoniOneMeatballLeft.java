package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;

public class TwoRigatoniOneMeatballLeft extends OrangeSequentialCommandGroup {
  public TwoRigatoniOneMeatballLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("TWO_RIGATONI_ONE_MEATBALL_LEFT");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              PathPlannerPath path = Robot.ThreeRigatoniStartToJuliet;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeRigatoniStartToJuliet),
        new ScoreRigatoni(superstructure, Level.L4, drive, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.JulietToIndiaJuliet),
        new DescoreMeatball(superstructure, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.MEATBALL_IDLE),
        AutoBuilder.followPath(Robot.IndiaJulietToLeftBarge),
        new MeatballPrescoreAuto(superstructure, drive),
        new WaitUntilCommand(
            () -> superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()),
        AutoBuilder.followPath(Robot.LeftBargeToLeftMeatballScore),
        new MeatballScoreAuto(superstructure, drive),
        new WaitCommand(0.2),
        new WaitUntilCommand(() -> !superstructure.isMeatballHeld()),
        new ParallelCommandGroup(
            new RigatoniIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.LeftMeatballScoreToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreRigatoni(superstructure, Level.L4, drive, false));
  }
}
