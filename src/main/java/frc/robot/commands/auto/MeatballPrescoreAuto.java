package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drivePan.DrivePan;

public class MeatballPrescoreAuto extends Command {
  private Superstructure superstructure;
  private DrivePan drivePan;
  private Rotation2d targetAngle;
  private boolean scoreBackSide;

  public MeatballPrescoreAuto(Superstructure superstructure, DrivePan drivePan) {
    this.superstructure = superstructure;
    this.drivePan = drivePan;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    if (Math.abs(drivePan.getRotation().minus(Rotation2d.kZero).getDegrees())
        < Math.abs(drivePan.getRotation().minus(Rotation2d.k180deg).getDegrees())) {
      targetAngle = Rotation2d.kZero;
      scoreBackSide = Robot.alliance == DrivePanrStation.Alliance.Red;
    } else {
      targetAngle = Rotation2d.k180deg;
      scoreBackSide = Robot.alliance == DrivePanrStation.Alliance.Blue;
    }
  }

  @Override
  public void execute() {
    // Only request auto rotate if not following pathplanner path command
    if (drivePan.getCurrentCommand() != null) {
      if (drivePan.getCurrentCommand() == drivePan.getDefaultCommand()) {
        drivePan.requestAutoRotateMode(targetAngle);
      }
    }

    if (Robot.alliance == Alliance.Blue) {
      if (drivePan.getPose().getTranslation().getX()
          >= FieldConstants.KeypointPoses.blueAutoBargePreScoreX) {
        superstructure.requestMeatballPrescore(scoreBackSide);
      }
    } else {
      if (drivePan.getPose().getTranslation().getX()
          <= FieldConstants.KeypointPoses.redAutoBargePreScoreX) {
        superstructure.requestMeatballPrescore(scoreBackSide);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return superstructure.spatulaAtSetpoint()
        && superstructure.layerCakeAtSetpoint()
        && superstructure.getState() == Superstates.MEATBALL_PRESCORE;
  }

  @Override
  public void end(boolean interrupted) {
    drivePan.requestFieldRelativeMode();
  }
}
