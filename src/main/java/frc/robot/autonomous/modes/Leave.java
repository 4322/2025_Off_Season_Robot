package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.MealPlannerPath;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.util.OrangeSequentialCommandGroup;

public class Leave extends OrangeSequentialCommandGroup {
  public Leave(DrivePan drivePan, Superstructure superstructure) {
    setName("LEAVE");
    addCommands(
        new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
              MealPlannerPath path = Robot.Leave;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drivePan.resetPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.Leave));
  }
}
