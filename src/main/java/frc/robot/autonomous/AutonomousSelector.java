package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  public AutonomousSelector(
      Drive drive, Superstructure superstructure, IntakeSuperstructure intakeSuperstructure) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());

    SmartDashboard.putData("Autonomous Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
