package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.CoralIntakeManualAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class TestAuto extends Command {
  private Superstructure superstructure;
  private Drive drive;
  private IntakeSuperstructure intakeSuperstructure;
  private Vision vision;

  private ScoreCoral scoreCoral1;
    private ScoreCoral scoreCoral2;
    private CoralIntakeManualAuto coralIntakeManualAuto;
  private Command ThreeCoralStartToJuliet;
  private Command JulietToFeed;
    private Command FeedToKilo;


  private int currentCommand = 0;

  private Command[] commands = new Command[5];

  public TestAuto(
      Superstructure superstructure,
      Drive drive,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    this.superstructure = superstructure;
    this.drive = drive;
    this.intakeSuperstructure = intakeSuperstructure;
    this.vision = vision;

    this.scoreCoral1 = new ScoreCoral(superstructure, Level.L4, drive);
    this.scoreCoral2 = new ScoreCoral(superstructure, Level.L4, drive);
    this.ThreeCoralStartToJuliet = AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet);
    this.coralIntakeManualAuto = new CoralIntakeManualAuto(intakeSuperstructure, true);
    this.JulietToFeed = AutoBuilder.followPath(Robot.JulietToFeed);

    this.commands[0] = ThreeCoralStartToJuliet;
    this.commands[1] = scoreCoral1;
    this.commands[2] = new ParallelCommandGroup(
        coralIntakeManualAuto,
        JulietToFeed
    );
    this.commands[3] = FeedToKilo;
    this.commands[4] = scoreCoral2;

    }
    
  

  @Override
  public void initialize() {
    

    superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
    PathPlannerPath path = Robot.ThreeCoralStartToJuliet;
    if (Robot.alliance == Alliance.Red) {
      path = path.flipPath();
    }
    drive.resetPose(path.getStartingHolonomicPose().get());
    commands[0].schedule();
    
  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
