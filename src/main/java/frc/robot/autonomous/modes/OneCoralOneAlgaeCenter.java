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
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class OneCoralOneAlgaeCenter extends SequentialCommandGroup {
  public OneCoralOneAlgaeCenter(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("ONE_CORAL_ONE_ALGAE_CENTER");
    addRequirements(drive, superstructure, intakeSuperstructure);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.CenterStartToGulf),
        new ScoreCoral(superstructure, Level.L4, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.GulfToGulfHotel),
        new DescoreAlgae(superstructure, Level.L2, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.ALGAE_IDLE),
        new ParallelCommandGroup(
            AutoBuilder.followPath(Robot.GulfHotelToCenterBarge),
            new AlgaePrescoreAuto(superstructure, drive)),
        new WaitUntilCommand(
            () -> superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()),
        AutoBuilder.followPath(Robot.CenterBargeToCenterAlgaeScore),
        new AlgaeScoreAuto(superstructure, drive),
        new WaitCommand(0.2),
        new WaitUntilCommand(() -> !superstructure.isAlgaeHeld()),
        AutoBuilder.followPath(Robot.CenterAlgaeScoreToLeave),
        new InstantCommand(() -> superstructure.requestIdle()));
  }
}
