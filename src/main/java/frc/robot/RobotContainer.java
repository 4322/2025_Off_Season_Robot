package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.CoastCommand;
import frc.robot.commands.DescoreAlgae;
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
import frc.robot.subsystems.arm.ArmIOSim;
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
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIONitrate;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIONitrate;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIONitrate;
import frc.robot.subsystems.vision.Vision;
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
  private static CommandXboxController sim1;

  private static Vision vision;
  private static Drive drive;
  private static Arm arm; // IO for the arm subsystem, null if not enabled
  // Declare Arm variable

  private static EndEffector endEffector;
  private static Indexer indexer;
  private static Rollers rollers;
  private static Deployer deployer;
  private static IntakeSuperstructure intakeSuperstructure;
  private static Superstructure superstructure;
  private static Elevator elevator;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    // create default non-existant subsystems
    arm = new Arm(new ArmIO() {});
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    elevator = new Elevator(new ElevatorIO() {});
    endEffector = new EndEffector(new EndEffectorIO() {});
    indexer = new Indexer(new IndexerIO() {});
    rollers = new Rollers(new RollersIO() {});
    deployer = new Deployer(new DeployerIO() {});

    switch (Constants.currentMode) {
      case REAL:
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
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        indexer = new Indexer(new IndexerIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        sim1 = new CommandXboxController(1);
        break;

      case REPLAY:
        break;
    }

    intakeSuperstructure = new IntakeSuperstructure(endEffector, deployer, rollers, indexer);
    superstructure = new Superstructure(endEffector, arm, elevator, intakeSuperstructure);

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

    driver.povUp().whileTrue(new Eject(intakeSuperstructure, superstructure));
    // Prescore/Descore Levels
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    superstructure.requestIntakeAlgaeFloor();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure, Level.L1).schedule();
                  }
                }));
    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    new DescoreAlgae(superstructure, Level.L2).schedule();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure, Level.L2).schedule();
                  }
                }));
    driver
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    new DescoreAlgae(superstructure, Level.L3).schedule();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure, Level.L3).schedule();
                  }
                }));
    driver
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!endEffector.hasCoral() && endEffector.hasAlgae()) {
                    new AlgaeScoreCommand(superstructure).schedule();
                  } else if (endEffector.hasCoral() && !endEffector.hasAlgae()) {
                    new ScoreCoral(superstructure, Level.L4).schedule();
                  }
                }));
    driver.leftStick().onTrue(new SwitchOperationModeCommand(superstructure));
    driver
        .back()
        .onTrue(
            new CoastCommand(arm, elevator, deployer)
                .onlyIf(() -> DriverStation.isDisabled())
                .ignoringDisable(true));
  }

  public static boolean isScoringTriggerHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return sim1.a().getAsBoolean();
    } else {
      return driver.rightTrigger().getAsBoolean();
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
    return Commands.none();
  }
}
