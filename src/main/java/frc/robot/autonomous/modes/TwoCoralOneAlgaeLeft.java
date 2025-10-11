package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.CoralIntakeManual;
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
import frc.robot.commands.auto.CoralIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class TwoCoralOneAlgaeLeft extends SequentialCommandGroup {
  public TwoCoralOneAlgaeLeft(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("TWO_CORAL_ONE_ALGAE_LEFT");
    addRequirements(drive, superstructure, intakeSuperstructure);
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
        new ScoreCoral(superstructure, Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.JulietToIndiaJuliet),
        new DescoreAlgae(superstructure, Level.L3, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.ALGAE_IDLE),
        AutoBuilder.followPath(Robot.IndiaJulietToLeftBarge),
        new AlgaePrescoreAuto(superstructure, drive),
        new WaitUntilCommand(
            () -> superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()),
        AutoBuilder.followPath(Robot.LeftBargeToLeftAlgaeScore),
        new AlgaeScoreAuto(superstructure, drive),
        new WaitCommand(0.2),
        new WaitUntilCommand(() -> !superstructure.isAlgaeHeld()),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure),
            AutoBuilder.followPath(Robot.LeftAlgaeScoreToFeed)),
        AutoBuilder.followPath(Robot.FeedToKilo),
        new ScoreCoral(superstructure, Level.L4, drive));
  }
}
