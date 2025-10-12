package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.auto.AlgaePrescoreAuto;
import frc.robot.commands.auto.AlgaeScoreAuto;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class OneCoralTwoAlgaeLeft extends SequentialCommandGroup {
    private Superstructure superstructure;
    private IntakeSuperstructure intakeSuperstructure;
    private Drive drive;
    private Vision vision;
    
    public OneCoralTwoAlgaeLeft(Superstructure superstructure, IntakeSuperstructure intakeSuperstructure, Vision vision, Drive drive) {
        this.superstructure = superstructure;
        this.intakeSuperstructure = intakeSuperstructure;
        this.drive = drive;
        this.vision = vision;

        addCommands(
            new InstantCommand(
            () -> {
              superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
              PathPlannerPath path = Robot.CenterStartToGulf;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.resetPose(path.getStartingHolonomicPose().get());
            }),
            AutoBuilder.followPath(Robot.ThreeCoralStartToJuliet),
            new ScoreCoral(superstructure, Level.L4, drive),
            AutoBuilder.followPath(Robot.JulietToIndiaJuliet),
            new DescoreAlgae(superstructure, Level.L2, drive),
            new ParallelCommandGroup(
                AutoBuilder.followPath(Robot.IndiaJulietToLeftBargeBackwards),
                new AlgaePrescoreAuto(superstructure, drive)
            ),
            AutoBuilder.followPath(Robot.LeftBargeBackwardsToLeftAlgaeScoreBackwards),
            new AlgaeScoreAuto(superstructure, drive),
            new WaitCommand(Constants.Auto.algaeScoreDelay),
            AutoBuilder.followPath(Robot.LeftAlgaeScoreBackwardsToKiloLima),
            new DescoreAlgae(superstructure, Level.L3, drive),
            AutoBuilder.followPath(Robot.KiloLimaToLeftBargeBackwards),
            new AlgaePrescoreAuto(superstructure, drive),
            AutoBuilder.followPath(Robot.LeftBargeBackwardsToLeftAlgaeScoreBackwards),
            new WaitCommand(Constants.Auto.algaeScoreDelay),
            AutoBuilder.followPath(Robot.LeftAlgaeScoreBackwardsToLeave)


        );
    }
}
