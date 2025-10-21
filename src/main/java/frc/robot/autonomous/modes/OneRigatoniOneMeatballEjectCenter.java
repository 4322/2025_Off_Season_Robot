package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.auto.EjectAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneRigatoniOneMeatballEjectCenter extends OrangeSequentialCommandGroup {
  public OneRigatoniOneMeatballEjectCenter(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("ONE_RIGATONI_ONE_MEATBALL_CENTER");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              PathPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.CenterStartToGulf),
        new ScoreRigatoni(superstructure, Level.L4, drive, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.GulfToGulfHotel),
        new DescoreMeatball(superstructure, drive),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.MEATBALL_IDLE),
        AutoBuilder.followPath(Robot.GulfHotelToCenterEject),
        new EjectAuto(intakeSuperstructure, superstructure, EjectAuto.EjectType.END_EFFECTOR, 1.0));
  }
}
