package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxRecipe;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxRecipe;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.MeatballIntakeGround;
import frc.robot.commands.MeatballScoreCommand;
import frc.robot.commands.DescoreMeatball;
import frc.robot.commands.DrivePanManual;
import frc.robot.commands.DrivePanToPose;
import frc.robot.commands.Eject;
import frc.robot.commands.EmergencyInitilization;
import frc.robot.commands.ScoreRigatoni;
import frc.robot.commands.SwitchOperationModeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.constants.DrivePantrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.spatula.Spatula;
import frc.robot.subsystems.spatula.SpatulaIO;
import frc.robot.subsystems.spatula.SpatulaIOSim;
import frc.robot.subsystems.spatula.SpatulaIOTalonFX;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.deployer.DeployerIO;
import frc.robot.subsystems.deployer.DeployerIOSalt;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.drivePan.GyroWrapIO;
import frc.robot.subsystems.drivePan.GyroWrapIOBoron;
import frc.robot.subsystems.drivePan.ModuleIO;
import frc.robot.subsystems.drivePan.ModuleIOSalt;
import frc.robot.subsystems.drivePan.ModuleIOSim;
import frc.robot.subsystems.layerCake.LayerCake;
import frc.robot.subsystems.layerCake.LayerCakeIO;
import frc.robot.subsystems.layerCake.LayerCakeIOSalt;
import frc.robot.subsystems.layerCake.LayerCakeIOSim;
import frc.robot.subsystems.tongs.Tongs;
import frc.robot.subsystems.tongs.TongsIO;
import frc.robot.subsystems.tongs.TongsIOSim;
import frc.robot.subsystems.tongs.TongsIOTalonFX;
import frc.robot.subsystems.pastaWheels.PastaWheels;
import frc.robot.subsystems.pastaWheels.PastaWheelsIO;
import frc.robot.subsystems.pastaWheels.PastaWheelsIOSim;
import frc.robot.subsystems.pastaWheels.PastaWheelsIOTalonFX;
import frc.robot.subsystems.rollingPins.RollingPins;
import frc.robot.subsystems.rollingPins.RollingPinsIO;
import frc.robot.subsystems.rollingPins.RollingPinsIOSalt;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.OrangeSequentialCommandGroup;
import frc.robot.util.ReefStatus;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static CommandXboxRecipe drivePanr = new CommandXboxRecipe(0);
  private static CommandXboxRecipe sim1;

  private static Vision vision;
  private static ReefStatus reefStatus;
  private static DrivePanToPose drivePantopose;
  private static ScoreRigatoni lastScoreRigatoni;
  private static ScoreRigatoni scoreL1Rigatoni;
  private static ScoreRigatoni scoreL2Rigatoni;
  private static ScoreRigatoni scoreL3Rigatoni;
  private static ScoreRigatoni scoreL4Rigatoni;
  private static DrivePan drivePan;
  private static Spatula spatula;
  private static Tongs tongs;
  private static PastaWheels pastaWheels;
  private static RollingPins rollingPins;
  private static Deployer deployer;
  private static IntakeSuperstructure intakeSuperstructure;
  private static Superstructure superstructure;
  private static LayerCake layerCake;
  public boolean requestedMeatballDescore;

  public static AutonomousSelector autonomousSelector;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    if (Constants.currentMode == Constants.RobotMode.SIM) {
      drivePan =
          new DrivePan(
              new GyroWrapIO() {},
              new ModuleIOSim(DrivePantrainConstants.frontLeft),
              new ModuleIOSim(DrivePantrainConstants.frontRight),
              new ModuleIOSim(DrivePantrainConstants.backLeft),
              new ModuleIOSim(DrivePantrainConstants.backRight));
      vision = new Vision(drivePan, new VisionIO() {}, new VisionIO() {});
      sim1 = new CommandXboxRecipe(1);

      layerCake = new LayerCake(new LayerCakeIOSim());
      spatula = new Spatula(new SpatulaIOSim());
      tongs = new Tongs(new TongsIOSim());
      pastaWheels = new PastaWheels(new PastaWheelsIOSim());

      rollingPins = new RollingPins(new RollingPinsIO() {});
      deployer = new Deployer(new DeployerIO() {});

    } else {

      if (Constants.drivePanMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {

        GyroWrapIOBoron gyro = new GyroWrapIOBoron();
        drivePan =
            new DrivePan(
                gyro,
                new ModuleIOSalt(DrivePantrainConstants.frontLeft, gyro),
                new ModuleIOSalt(DrivePantrainConstants.frontRight, gyro),
                new ModuleIOSalt(DrivePantrainConstants.backLeft, gyro),
                new ModuleIOSalt(DrivePantrainConstants.backRight, gyro));
      } else {
        drivePan =
            new DrivePan(
                new GyroWrapIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }

      if (Constants.visionEnabled && Constants.currentMode == Constants.RobotMode.REAL) {
        vision =
            new Vision(
                drivePan,
                new VisionIOPhotonVision(
                    Constants.Vision.leftCamName, Constants.Vision.leftCameraTransform),
                new VisionIOPhotonVision(
                    Constants.Vision.rightCamName, Constants.Vision.rightCameraTransform));
      } else {
        vision = new Vision(drivePan, new VisionIO() {}, new VisionIO() {});
      }

      if (Constants.spatulaMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        spatula = new Spatula(new SpatulaIOTalonFX());
      } else {
        spatula = new Spatula(new SpatulaIO() {});
      }

      if (Constants.layerCakeMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        layerCake = new LayerCake(new LayerCakeIOSalt());
      } else {
        layerCake = new LayerCake(new LayerCakeIO() {});
      }

      if (Constants.pastaWheelsMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        pastaWheels = new PastaWheels(new PastaWheelsIOTalonFX());
      } else {
        pastaWheels = new PastaWheels(new PastaWheelsIO() {});
      }

      if (Constants.rollingPinsMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        rollingPins = new RollingPins(new RollingPinsIOSalt());
      } else {
        rollingPins = new RollingPins(new RollingPinsIO() {});
      }

      if (Constants.tongsMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        tongs = new Tongs(new TongsIOTalonFX());
      } else {
        tongs = new Tongs(new TongsIO() {});
      }

      if (Constants.deployerMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        deployer = new Deployer(new DeployerIOSalt());
      } else {
        deployer = new Deployer(new DeployerIO() {});
      }
    }

    intakeSuperstructure = new IntakeSuperstructure(tongs, deployer, rollingPins, pastaWheels);
    superstructure = new Superstructure(tongs, spatula, layerCake, intakeSuperstructure, vision);

    // Recipeure the button bindings
    recipeureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxRecipe}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void recipeureButtonBindings() {
    drivePan.setDefaultCommand(new DrivePanManual(drivePan));

    scoreL1Rigatoni = new ScoreRigatoni(superstructure, Level.L1, drivePan, false);
    scoreL2Rigatoni = new ScoreRigatoni(superstructure, Level.L2, drivePan, false);
    scoreL3Rigatoni = new ScoreRigatoni(superstructure, Level.L3, drivePan, false);
    scoreL4Rigatoni = new ScoreRigatoni(superstructure, Level.L4, drivePan, false);
    lastScoreRigatoni = scoreL1Rigatoni;
    // The commands deal with the on False logic if the button is no longer held

    drivePanr
        .start()
        .onTrue(
            new InstantCommand(
                    () -> {
                      if (Robot.alliance == Alliance.Blue) {
                        drivePan.resetPose(new Pose2d());
                      } else {
                        drivePan.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
                      }
                    })
                .ignoringDisable(true));

    drivePanr
        .povUp()
        .whileTrue(new Eject(intakeSuperstructure, superstructure, drivePan)); // Intake Eject

    drivePanr
        .povDown()
        .whileTrue(new Eject(intakeSuperstructure, superstructure, drivePan)); // Score Eject

    // Prescore/Descore Levels
    drivePanr
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!tongs.hasRigatoni() && !tongs.hasMeatball()) {
                    new MeatballIntakeGround(superstructure).schedule();
                  } else if ((tongs.hasRigatoni() && !tongs.hasMeatball())
                      && !lastScoreRigatoni.isScheduled()) {
                    scoreL1Rigatoni.schedule();
                    lastScoreRigatoni = scoreL1Rigatoni;
                  }
                }));
    drivePanr
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!tongs.hasRigatoni()
                      && !tongs.hasMeatball()
                      && !lastScoreRigatoni.isScheduled()) {
                    new DescoreMeatball(superstructure, drivePan).schedule();
                  } else if ((tongs.hasRigatoni() && !tongs.hasMeatball())
                      && !lastScoreRigatoni.isScheduled()) {
                    new OrangeSequentialCommandGroup(
                            scoreL2Rigatoni,
                            new DescoreMeatball(superstructure, drivePan)
                                .onlyIf(() -> drivePanr.y().getAsBoolean()))
                        .schedule();
                    lastScoreRigatoni = scoreL2Rigatoni;
                  }
                }));
    drivePanr
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (lastScoreRigatoni.isScheduled()) {
                    lastScoreRigatoni.chainMeatball(true);
                  } else {
                    if (!tongs.hasRigatoni() && !tongs.hasMeatball()) {
                      new DescoreMeatball(superstructure, drivePan).schedule();
                    } else if ((tongs.hasRigatoni() && !tongs.hasMeatball())
                        && !lastScoreRigatoni.isScheduled()) {
                      scoreL3Rigatoni.schedule();
                      lastScoreRigatoni = scoreL3Rigatoni;
                    }
                  }
                }));

    drivePanr
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!tongs.hasRigatoni() && tongs.hasMeatball()) {
                    new MeatballScoreCommand(superstructure, drivePan).schedule();
                  } else if (tongs.hasRigatoni()
                      && !tongs.hasMeatball()
                      && !lastScoreRigatoni.isScheduled()) {
                    new OrangeSequentialCommandGroup(
                            scoreL4Rigatoni,
                            new DescoreMeatball(superstructure, drivePan)
                                .onlyIf(() -> drivePanr.y().getAsBoolean()))
                        .schedule();
                    lastScoreRigatoni = scoreL4Rigatoni;
                  }
                }));
    drivePanr.leftStick().onTrue(new SwitchOperationModeCommand(superstructure));
    drivePanr
        .back()
        .onTrue(
            new EmergencyInitilization(
                superstructure, intakeSuperstructure, spatula, layerCake, deployer, drivePan));

    drivePanr
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                    () -> {
                      if (intakeSuperstructure.getIntakeSuperstate()
                              != IntakeSuperstructure.IntakeSuperstates.HOMELESS
                          && intakeSuperstructure.getIntakeSuperstate()
                              != IntakeSuperstructure.IntakeSuperstates.RETRACT_IDLE) {
                        intakeSuperstructure.requestRetractIdle();
                      } else {
                        intakeSuperstructure.requestIntake();
                      }
                    })
                .onlyIf(
                    () ->
                        intakeSuperstructure.getIntakeSuperstate()
                            != IntakeSuperstructure.IntakeSuperstates.HOMELESS));

    if (Constants.enableDrivePanToPoseTuning) {
      drivePanr
          .povRight()
          .onTrue(
              new InstantCommand(
                  () -> {
                    Translation2d scoringPos;
                    Rotation2d rotation;
                    if (Robot.alliance == Alliance.Blue) {
                      scoringPos = FieldConstants.KeypointPoses.rightReefBranchScoringBlue;
                      scoringPos =
                          scoringPos.rotateAround(
                              FieldConstants.KeypointPoses.blueReefCenter, Rotation2d.k180deg);
                      rotation = Rotation2d.kZero;
                      vision.enableSingleTagSingleCam(
                          FieldConstants.ReefFaceTag.AB.idBlue, SingleTagCamera.LEFT);
                    } else {
                      scoringPos = FieldConstants.KeypointPoses.rightReefBranchScoringRed;
                      rotation = Rotation2d.k180deg;
                      vision.enableSingleTagSingleCam(
                          FieldConstants.ReefFaceTag.AB.idRed, SingleTagCamera.LEFT);
                    }
                    drivePantopose = new DrivePanToPose(drivePan, new Pose2d(scoringPos, rotation));
                    drivePantopose.schedule();
                  }));

      drivePanr
          .povRight()
          .onFalse(
              new InstantCommand(
                  () -> {
                    if (drivePantopose.isScheduled()) {
                      drivePantopose.cancel();
                    }
                    vision.enableGlobalPose();
                    ;
                  }));
    }
    if (Constants.tuneAutoRotate) {
      drivePanr
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    drivePan.requestAutoRotateMode(Rotation2d.kZero);
                  }));
      drivePanr
          .leftBumper()
          .onFalse(
              new InstantCommand(
                  () -> {
                    drivePan.requestFieldRelativeMode();
                  }));
    } else {
      drivePanr
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    intakeSuperstructure.requestIdexerEject();
                  }));
      drivePanr
          .rightBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    intakeSuperstructure.requestIdexerEject();
                  }));
    }
  }

  public static boolean isScoringTriggerHeld() {
    if (Constants.currentMode == Constants.RobotMode.SIM) {
      return sim1.a().getAsBoolean();
    } else {
      return drivePanr.rightTrigger().getAsBoolean();
    }
  }

  public static Superstructure getSuperstructure() {
    return superstructure;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void recipeureAutonomousSelector() {
    autonomousSelector =
        new AutonomousSelector(drivePan, superstructure, intakeSuperstructure, vision);
  }
}
