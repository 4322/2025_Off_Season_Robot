package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.AlgaeIntakeGround;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.DescoreAlgae;
import frc.robot.commands.DriveManual;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.Eject;
import frc.robot.commands.EmergencyInitilization;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SwitchOperationModeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.deployer.DeployerIO;
import frc.robot.subsystems.deployer.DeployerIONitrate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIONitrate;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIONitrate;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.OrangeSequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static CommandXboxController driver = new CommandXboxController(0);
  private static CommandXboxController sim1;

  private static Vision vision;
  private static DriveToPose drivetopose;
  private static ScoreCoral lastScoreCoral;
  private static ScoreCoral scoreL1Coral;
  private static ScoreCoral scoreL2Coral;
  private static ScoreCoral scoreL3Coral;
  private static ScoreCoral scoreL4Coral;
  private static Drive drive;
  private static Arm arm;
  private static EndEffector endEffector;
  private static Indexer indexer;
  private static Rollers rollers;
  private static Deployer deployer;
  private static IntakeSuperstructure intakeSuperstructure;
  private static Superstructure superstructure;
  private static Elevator elevator;
  public boolean requestedAlgaeDescore;
  private Timer batteryLowStopTimer = new Timer();

  public static AutonomousSelector autonomousSelector;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    if (Constants.currentMode == Constants.RobotMode.SIM) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(DrivetrainConstants.frontLeft),
              new ModuleIOSim(DrivetrainConstants.frontRight),
              new ModuleIOSim(DrivetrainConstants.backLeft),
              new ModuleIOSim(DrivetrainConstants.backRight));
      vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
      sim1 = new CommandXboxController(1);

      elevator = new Elevator(new ElevatorIOSim());
      arm = new Arm(new ArmIOSim());
      endEffector = new EndEffector(new EndEffectorIOSim());
      indexer = new Indexer(new IndexerIOSim());

      rollers = new Rollers(new RollersIO() {});
      deployer = new Deployer(new DeployerIO() {});

    } else {

      if (Constants.driveMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {

        GyroIOBoron gyro = new GyroIOBoron();
        drive =
            new Drive(
                gyro,
                new ModuleIONitrate(DrivetrainConstants.frontLeft, gyro),
                new ModuleIONitrate(DrivetrainConstants.frontRight, gyro),
                new ModuleIONitrate(DrivetrainConstants.backLeft, gyro),
                new ModuleIONitrate(DrivetrainConstants.backRight, gyro));
      } else {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }

      if (Constants.visionEnabled && Constants.currentMode == Constants.RobotMode.REAL) {
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    Constants.Vision.leftCamName, Constants.Vision.leftCameraTransform),
                new VisionIOPhotonVision(
                    Constants.Vision.rightCamName, Constants.Vision.rightCameraTransform));
      } else {
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
      }

      if (Constants.armMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        arm = new Arm(new ArmIOTalonFX());
      } else {
        arm = new Arm(new ArmIO() {});
      }

      if (Constants.elevatorMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        elevator = new Elevator(new ElevatorIOTalonFX());
      } else {
        elevator = new Elevator(new ElevatorIO() {});
      }

      if (Constants.indexerMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        indexer = new Indexer(new IndexerIONitrate());
      } else {
        indexer = new Indexer(new IndexerIO() {});
      }

      if (Constants.rollersMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        rollers = new Rollers(new RollersIOTalonFX());
      } else {
        rollers = new Rollers(new RollersIO() {});
      }

      if (Constants.endEffectorMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        endEffector = new EndEffector(new EndEffectorIOTalonFX());
      } else {
        endEffector = new EndEffector(new EndEffectorIO() {});
      }

      if (Constants.deployerMode != SubsystemMode.DISABLED
          && Constants.currentMode == Constants.RobotMode.REAL) {
        deployer = new Deployer(new DeployerIONitrate());
      } else {
        deployer = new Deployer(new DeployerIO() {});
      }
    }

    intakeSuperstructure = new IntakeSuperstructure(endEffector, deployer, rollers, indexer);
    superstructure = new Superstructure(endEffector, arm, elevator, intakeSuperstructure, vision);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(new DriveManual(drive));

    if (RobotController.getBatteryVoltage() <= 12.00
        && (superstructure.getState() == Superstructure.Superstates.IDLE
            || superstructure.getState() == Superstructure.Superstates.CORAL_HELD)
        && intakeSuperstructure.getIntakeSuperstate()
            == IntakeSuperstructure.IntakeSuperstates.RETRACT_IDLE
        && !DriverStation.isFMSAttached()) {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, 0);
      batteryLowStopTimer.start();
      if (batteryLowStopTimer.hasElapsed(5)) {
        driver.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        batteryLowStopTimer.stop();
        batteryLowStopTimer.reset();
        DriverStation.reportError(
            "Battery voltage is critically low (" + RobotController.getBatteryVoltage() + "V)",
            false);
      }
    } else {
      batteryLowStopTimer.stop();
      batteryLowStopTimer.reset();
    }

    if (RobotController.getBatteryVoltage() > 12.00) {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    scoreL1Coral = new ScoreCoral(superstructure, Level.L1, drive, false);
    scoreL2Coral = new ScoreCoral(superstructure, Level.L2, drive, false);
    scoreL3Coral = new ScoreCoral(superstructure, Level.L3, drive, false);
    scoreL4Coral = new ScoreCoral(superstructure, Level.L4, drive, false);
    lastScoreCoral = scoreL1Coral;
    // The commands deal with the on False logic if the button is no longer held

    driver
        .start()
        .onTrue(
            new InstantCommand(
                    () -> {
                      if (Robot.alliance == Alliance.Blue) {
                        drive.resetPose(new Pose2d());
                      } else {
                        drive.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
                      }
                    })
                .ignoringDisable(true));

    driver
        .povUp()
        .whileTrue(new Eject(intakeSuperstructure, superstructure, drive)); // Intake Eject

    driver
        .povDown()
        .whileTrue(new Eject(intakeSuperstructure, superstructure, drive)); // Score Eject

    // Prescore/Descore Levels
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new AlgaeIntakeGround(superstructure).schedule();
                  } else if ((endEffector.hasCoral() && !endEffector.hasAlgae())
                      && !lastScoreCoral.isScheduled()) {
                    scoreL1Coral.schedule();
                    lastScoreCoral = scoreL1Coral;
                  }
                }));
    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral()
                      && !endEffector.hasAlgae()
                      && !lastScoreCoral.isScheduled()) {
                    new DescoreAlgae(superstructure, drive).schedule();
                  } else if ((endEffector.hasCoral() && !endEffector.hasAlgae())
                      && !lastScoreCoral.isScheduled()) {
                    new OrangeSequentialCommandGroup(
                            scoreL2Coral,
                            new DescoreAlgae(superstructure, drive)
                                .onlyIf(() -> driver.y().getAsBoolean()))
                        .schedule();
                    lastScoreCoral = scoreL2Coral;
                  }
                }));
    driver
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (lastScoreCoral.isScheduled()) {
                    lastScoreCoral.chainAlgae(true);
                  } else {
                    if (!endEffector.hasCoral() && !endEffector.hasAlgae()) {
                      new DescoreAlgae(superstructure, drive).schedule();
                    } else if ((endEffector.hasCoral() && !endEffector.hasAlgae())
                        && !lastScoreCoral.isScheduled()) {
                      scoreL3Coral.schedule();
                      lastScoreCoral = scoreL3Coral;
                    }
                  }
                }));

    driver
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    new AlgaeScoreCommand(superstructure, drive).schedule();
                  } else if (endEffector.hasCoral()
                      && !endEffector.hasAlgae()
                      && !lastScoreCoral.isScheduled()) {
                    new OrangeSequentialCommandGroup(
                            scoreL4Coral,
                            new DescoreAlgae(superstructure, drive)
                                .onlyIf(() -> driver.y().getAsBoolean()))
                        .schedule();
                    lastScoreCoral = scoreL4Coral;
                  }
                }));
    driver.leftStick().onTrue(new SwitchOperationModeCommand(superstructure));
    driver
        .back()
        .onTrue(
            new EmergencyInitilization(
                superstructure, intakeSuperstructure, arm, elevator, deployer, drive));

    driver
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

    if (Constants.enableDriveToPoseTuning) {
      driver
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
                    drivetopose = new DriveToPose(drive, new Pose2d(scoringPos, rotation));
                    drivetopose.schedule();
                  }));

      driver
          .povRight()
          .onFalse(
              new InstantCommand(
                  () -> {
                    if (drivetopose.isScheduled()) {
                      drivetopose.cancel();
                    }
                    vision.enableGlobalPose();
                    ;
                  }));
    }
    if (Constants.tuneAutoRotate) {
      driver
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    drive.requestAutoRotateMode(Rotation2d.kZero);
                  }));
      driver
          .leftBumper()
          .onFalse(
              new InstantCommand(
                  () -> {
                    drive.requestFieldRelativeMode();
                  }));
    } else {
      driver
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    intakeSuperstructure.requestIdexerEject();
                    superstructure.requestDropCoralRepickup();
                  }));
      driver
          .rightBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    intakeSuperstructure.requestIdexerEject();
                    superstructure.requestDropCoralRepickup();
                  }));
    }
  }

  public static boolean isScoringTriggerHeld() {
    if (Constants.currentMode == Constants.RobotMode.SIM) {
      return sim1.a().getAsBoolean();
    } else {
      return driver.rightTrigger().getAsBoolean();
    }
  }

  public static Superstructure getSuperstructure() {
    return superstructure;
  }

  public static IntakeSuperstructure getIntakeSuperstructure() {
    return intakeSuperstructure;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector =
        new AutonomousSelector(drive, superstructure, intakeSuperstructure, vision);
  }
}
