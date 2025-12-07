package frc.robot.autonomous;

import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.OneCoralTwoAlgaeCenter;
import frc.robot.autonomous.modes.TestLeave;
import frc.robot.autonomous.modes.ThreeCoralLeft;
import frc.robot.autonomous.modes.ThreeCoralRight;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Simulator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  public enum AutoName {
    DO_NOTHING,
    LEAVE,
    TEST_LEAVE,
    ONE_CORAL_TWO_ALGAE_CENTER,
    THREE_CORAL_LEFT,
    THREE_CORAL_RIGHT
  }

  private class Auto {
    AutoName name;
    SequentialCommandGroup command;

    private Auto(AutoName name, SequentialCommandGroup command) {
      this.name = name;
      this.command = command;
    }
  }

  private List<Auto> autos;
  private AutoName defaultAuto = AutoName.DO_NOTHING;

  public AutonomousSelector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision,
      VisionObjectDetection objectDetection) {
    autos =
        List.of(/* 
            new Auto(AutoName.DO_NOTHING, new DoNothing(superstructure)),
            new Auto(AutoName.LEAVE, new Leave(drive, superstructure)),
            new Auto(
                AutoName.ONE_CORAL_TWO_ALGAE_CENTER,
                new OneCoralTwoAlgaeCenter(drive, superstructure)),*/
            new Auto(
                AutoName.THREE_CORAL_LEFT,
                new ThreeCoralLeft(
                    drive, superstructure, intakeSuperstructure, vision, objectDetection)),
            new Auto(
                AutoName.THREE_CORAL_RIGHT,
                new ThreeCoralRight(
                    drive, superstructure, intakeSuperstructure, vision, objectDetection)));

    for (Auto nextAuto : autos) {
      if (nextAuto.name == defaultAuto) {
        autonomousSelector.addDefaultOption(nextAuto.name.toString(), nextAuto.command);
      } else {
        autonomousSelector.addOption(nextAuto.name.toString(), nextAuto.command);
      }
    }
    /* 
    if (Constants.wantDriveTestAutos) {
      autonomousSelector.addOption(
          AutoName.TEST_LEAVE.toString(), new TestLeave(drive, superstructure));
    }*/
  }

  public SequentialCommandGroup get() {
    if (Constants.currentMode == RobotMode.SIM) {
      for (Auto nextAuto : autos) {
        if (nextAuto.name == Simulator.simulatedAuto) {
          return nextAuto.command;
        }
      }
      System.out.println("Simulated auto " + Simulator.simulatedAuto + " not found");
      System.exit(1);
    }
    return autonomousSelector.get();
  }
}
