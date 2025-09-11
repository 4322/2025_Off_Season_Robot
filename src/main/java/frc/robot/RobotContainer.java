package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveManual;
import frc.robot.commands.Eject;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SwitchOperationModeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIONitrate;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.deployer.DeployerIO;
import frc.robot.subsystems.deployer.DeployerIONitrate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIONitrate;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIONitrate;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIONitrate;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIONitrate;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIONitrate;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static CommandXboxController driver = new CommandXboxController(0);
  public static XboxController test = new XboxController(1);

  private static Vision vision;
  private static Drive drive;
  private static Arm arm; // IO for the arm subsystem, null if not enabled
  // Declare Arm variable
  private Superstructure.Level level;

  private static EndEffector endEffector;
  private static Indexer indexer;
  private static Rollers rollers;
  private static Deployer deployer;
  private static IntakeSuperstructure intakeSuperstructure;
  private static Superstructure superstructure;
  private static Elevator elevator;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        initializeEmptySubsystems();

        // Real robot, instantiate hardware IO implementations
        if (Constants.armEnabled) {
          arm = new Arm(new ArmIONitrate()); // Create the arm subsystem if enabled
        }
        if (Constants.elevatorEnabled) {
          elevator =
              new Elevator(new ElevatorIONitrate()); // Create the elevator subsystem if enabled
        }
        if (Constants.driveEnabled) {
          GyroIOBoron gyro = new GyroIOBoron();
          drive =
              new Drive(
                  gyro,
                  new ModuleIONitrate(DrivetrainConstants.frontLeft, gyro),
                  new ModuleIONitrate(DrivetrainConstants.frontRight, gyro),
                  new ModuleIONitrate(DrivetrainConstants.backLeft, gyro),
                  new ModuleIONitrate(DrivetrainConstants.backRight, gyro));
        }
        if (Constants.visionEnabled) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));
        }
        if (Constants.endEffectorEnabled) {
          endEffector = new EndEffector(new EndEffectorIONitrate());
        }
        if (Constants.indexerEnabled) {
          indexer = new Indexer(new IndexerIONitrate());
        }
        if (Constants.rollersEnabled) {
          rollers = new Rollers(new RollersIONitrate());
        }
        if (Constants.deployerEnabled) {
          deployer = new Deployer(new DeployerIONitrate());
        }
        if (vision == null) {
          vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        }

        intakeSuperstructure = new IntakeSuperstructure(endEffector, deployer, rollers, indexer);
        superstructure =
            new Superstructure(
                endEffector, arm, indexer, elevator, drive, vision, intakeSuperstructure);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        initializeEmptySubsystems();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intakeSuperstructure = new IntakeSuperstructure(endEffector, deployer, rollers, indexer);
        superstructure =
            new Superstructure(
                endEffector, arm, indexer, elevator, drive, vision, intakeSuperstructure);
        break;

      default:
        break;
    }

    // Used during replay mode or when certain subsystems are disabled

    // Configure the button bindings
    configureButtonBindings();
  }

  private void initializeEmptySubsystems() {
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }

    if (endEffector == null) {
      endEffector = new EndEffector(new EndEffectorIO() {});
    }

    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }

    if (rollers == null) {
      rollers = new Rollers(new RollersIO() {});
    }

    if (deployer == null) {
      deployer = new Deployer(new DeployerIO() {});
    }

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
    // The commands deal with the on False logic if the button is no longer held

    driver
        .povUp()
        .whileTrue(
            new InstantCommand(
                () -> {
                  new Eject(intakeSuperstructure, superstructure);
                }));
    // Prescore/Descore Levels
    driver
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  level = Level.L1;
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    superstructure.requestIntakeAlgaeFloor();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure);
                  }
                }));
    driver
        .x()
        .whileTrue(
            new InstantCommand(
                () -> {
                  level = Level.L2;
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    superstructure.requestDescoreAlgae(Level.L2);
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure);
                  }
                }));
    driver
        .y()
        .whileTrue(
            new InstantCommand(
                () -> {
                  level = Level.L3;
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    superstructure.requestDescoreAlgae(Level.L3); // TODO DELETE
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure);
                  }
                }));
    driver
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  level = Level.L4;
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    superstructure.requestAlgaePrescore();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure);
                  }
                }));
    driver
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> { // Maybe has double logic (Get rid of if it works in the command)
                  if (endEffector.hasCoral()
                      && !endEffector.hasAlgae()
                      && !superstructure.isAutoOperationMode()) {
                    superstructure.requestScoreCoral(level);
                  } else if (!endEffector.hasCoral()
                      && endEffector.hasAlgae()) {
                    superstructure.requestAlgaeScore();
                  }
                }));
    driver
        .leftStick() // Figure out what to do with this because this is a tigger when we want it to
        // be a button
        .onTrue(
            new InstantCommand(
                    () -> {
                      new SwitchOperationModeCommand(superstructure);
                    })
                .ignoringDisable(false));
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
    return Commands.none();
  }
}
