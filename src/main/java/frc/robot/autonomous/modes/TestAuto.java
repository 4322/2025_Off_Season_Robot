package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class TestAuto extends Command {
  private Superstructure superstructure;
  private Drive drive;
  private IntakeSuperstructure intakeSuperstructure;
  private Vision vision;

  private ScoreCoral scoreCoral;
  private Command ThreeCoralStartToJuliet;

  private boolean started;

  public TestAuto(
      Superstructure superstructure,
      Drive drive,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    this.superstructure = superstructure;
    this.drive = drive;
    this.intakeSuperstructure = intakeSuperstructure;
    this.vision = vision;

    this.started = false;

    this.scoreCoral = new ScoreCoral(superstructure, Level.L4, drive);
    this.ThreeCoralStartToJuliet = AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet);
  }

  @Override
  public void initialize() {
    superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
    PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
    if (Robot.alliance == Alliance.Red) {
      path = path.flipPath();
    }
    drive.resetPose(path.getStartingHolonomicPose().get());
    ThreeCoralStartToJuliet.schedule();
  }

  @Override
  public void execute() {
    if (ThreeCoralStartToJuliet.isFinished()) {
      if (!scoreCoral.isScheduled()) {
        scoreCoral.schedule();
        started = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return scoreCoral.isFinished() && started;
  }
}
