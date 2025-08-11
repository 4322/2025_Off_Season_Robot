package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.deployer.DeployerIO;
import frc.robot.subsystems.deployer.DeployerIONitrate;
import frc.robot.subsystems.drive.DemoDrive;
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
  public static XboxController driver = new XboxController(0);

  private static Vision vision;
  private static DemoDrive drive = new DemoDrive(); // Demo drive subsystem, sim only
  private static Arm arm; // IO for the arm subsystem, null if not enabled
  // Declare Arm variable

  public static EndEffectorIO endEffectorIO =
      Constants.endEffectorEnabled ? new EndEffectorIONitrate() : new EndEffectorIO() {};
  public static IndexerIO indexerIO =
      Constants.indexerEnabled ? new IndexerIONitrate() : new IndexerIO() {};
  public static RollersIO rollersIO =
      Constants.rollersEnabled ? new RollersIONitrate() : new RollersIO() {};
  public static DeployerIO deployerIO =
      Constants.deployerEnabled ? new DeployerIONitrate() : new DeployerIO() {};

  public static EndEffector endEffector = new EndEffector(endEffectorIO);
  public static Indexer indexer = new Indexer(indexerIO);
  public static Rollers rollers = new Rollers(rollersIO);
  public static Deployer deployer = new Deployer(deployerIO);
  public static Superstructure superstructure = new Superstructure();

  public static IntakeSuperstructure intakeSuperstructure =
      new IntakeSuperstructure(endEffector, deployer, rollers, indexer, superstructure);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (Constants.armEnabled) {
          arm = new Arm(); // Create the arm subsystem if enabled
        }
        if (Constants.visionEnabled) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        break;
    }

    // Used during replay mode or when certain subsystems are disabled
    if (vision == null) {
      vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
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
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
