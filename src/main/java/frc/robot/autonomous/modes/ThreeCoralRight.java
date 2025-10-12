package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.CoralIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ThreeCoralRight extends SequentialCommandGroup {

  private Supplier<Pose2d> poseRequest1 = () -> new Pose2d();
  private Supplier<Pose2d> poseRequest2 = () -> new Pose2d();
  private Supplier<Pose2d> poseRequest3 = () -> new Pose2d();
  private BooleanSupplier booleanSupplier1;
  private BooleanSupplier booleanSupplier2;
  private BooleanSupplier booleanSupplier3;

  public ThreeCoralRight(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("THREE_CORAL_RIGHT");
    addRequirements(drive, superstructure, intakeSuperstructure);
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
              PathPlannerPath path = Robot.ThreeCoralStartToEcho;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToEcho),
        new InstantCommand(() -> Logger.recordOutput("Auto", "Finished path")),
        new ParallelRaceGroup(
          new DriveToPose(drive, () -> poseRequest1.get(), booleanSupplier1),
          new ScoreCoral(poseRequest1, booleanSupplier1.getAsBoolean(), superstructure, Level.L4, drive)),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.EchoToFeed)),
        AutoBuilder.followPath(Robot.FeedToDelta),
        new ParallelRaceGroup(
          new DriveToPose(drive, () -> poseRequest1.get(), booleanSupplier1),
          new ScoreCoral(poseRequest2, booleanSupplier2.getAsBoolean(), superstructure, Level.L4, drive)),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new ParallelCommandGroup(
            new CoralIntakeManualAuto(intakeSuperstructure, true),
            AutoBuilder.followPath(Robot.DeltaToFeed)),
        AutoBuilder.followPath(Robot.FeedToCharlie),
        new ParallelRaceGroup(
          new DriveToPose(drive, () -> poseRequest1.get(), booleanSupplier1),
          new ScoreCoral(poseRequest3, booleanSupplier3.getAsBoolean(), superstructure, Level.L4, drive)));
  }
}
