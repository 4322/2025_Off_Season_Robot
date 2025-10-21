package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
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
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;

public class OneRigatoniOneMeatballEjectCenter extends OrangeSequentialCommandGroup {
  public OneRigatoniOneMeatballEjectCenter(
      DrivePan drivePan,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    setName("ONE_RIGATONI_ONE_MEATBALL_CENTER");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              MealPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drivePan.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.CenterStartToGulf),
        new ScoreRigatoni(superstructure, Level.L4, drivePan, false),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.GulfToGulfHotel),
        new DescoreMeatball(superstructure, drivePan),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.MEATBALL_IDLE),
        AutoBuilder.followPath(Robot.GulfHotelToCenterEject),
        new EjectAuto(intakeSuperstructure, superstructure, EjectAuto.EjectType.END_EFFECTOR, 1.0));
  }
}
