package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // Values for Cu60 DCMotor specs come from https://docs.reduxrobotics.com/cu60/specifications
  private static final RobotConfig pathPlannerConfig =
      new RobotConfig(
          Constants.PathPlanner.robotMassKg,
          Constants.PathPlanner.robotMOI,
          new ModuleConfig(
              DrivetrainConstants.frontLeft.driveWheelRadius,
              DrivetrainConstants.maxSpeedAt12Volts,
              Constants.PathPlanner.wheelCOF,
              new DCMotor(
                      12.0, 7.3, 440.0, 2.0, Units.rotationsPerMinuteToRadiansPerSecond(6780), 1)
                  .withReduction(DrivetrainConstants.frontLeft.driveMotorGearRatio),
              DrivetrainConstants.frontLeft
                  .driveElectricalLimitSettings
                  .getStatorCurrentLimit()
                  .get(),
              1),
          getModuleTranslations());

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveDrivePoseEstimator poseEstimator;

  private ManualDriveMode manualDriveMode = ManualDriveMode.FIELD_RELATIVE;
  private Rotation2d targetAutoRotateAngle = Rotation2d.kZero;

  public enum ManualDriveMode {
    FIELD_RELATIVE,
    AUTO_ROTATE
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, DrivetrainConstants.frontLeft);
    modules[1] = new Module(frModuleIO, 1, DrivetrainConstants.frontRight);
    modules[2] = new Module(blModuleIO, 2, DrivetrainConstants.backLeft);
    modules[3] = new Module(brModuleIO, 3, DrivetrainConstants.backRight);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              modules[0].getPosition(),
              modules[1].getPosition(),
              modules[2].getPosition(),
              modules[3].getPosition()
            },
            Pose2d.kZero,
            VecBuilder.fill(0.3, 0.3, 0.05),
            VecBuilder.fill(0.4, 0.4, 0.4));

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds) -> runVelocity(speeds, false),
        new PPHolonomicDriveController(
            new PIDConstants(
                Constants.PathPlanner.translationkP, 0.0, Constants.PathPlanner.translationkD),
            new PIDConstants(Constants.PathPlanner.rotkP, 0.0, Constants.PathPlanner.rotkD)),
        pathPlannerConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    poseEstimator.updateWithTime(
        RobotController.getFPGATime() / 1e6, gyroInputs.yawAngle, getModulePositions());

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }

  /**
   * Only to be used in commands which directly control drive subsystem such as DriveManual or
   * DriveToPoint.
   *
   * <p>For commands requiring auto rotate capability or switch back to regular field relative
   * driving, use drive mode request methods instead.
   */
  public void runVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DrivetrainConstants.maxSpeedAt12Volts);

    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runClosedLoopDrive(setpointStates[i]);
    }

    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Only to be used in commands which directly control drive subsystem such as DriveManual or
   * DriveToPoint.
   *
   * <p>For commands requiring auto rotate capability or switch back to regular field relative
   * driving, use drive mode request methods instead.
   */
  public void runOpenLoop(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
    }
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DrivetrainConstants.maxSpeedAt12Volts);

    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runOpenLoopDrive(setpointStates[i]);
    }

    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void requestFieldRelativeMode() {
    manualDriveMode = ManualDriveMode.FIELD_RELATIVE;
  }

  public void requestAutoRotateMode(Rotation2d fieldTargetAngle) {
    manualDriveMode = ManualDriveMode.AUTO_ROTATE;
    targetAutoRotateAngle = fieldTargetAngle;
  }

  public ManualDriveMode getManualDriveMode() {
    return manualDriveMode;
  }

  public Rotation2d getTargetAngle() {
    return targetAutoRotateAngle;
  }

  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRotation());
  }

  public double getYawVelocity() {
    return gyroInputs.yawVelocityDegPerSec;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public boolean atAngularSetpoint(double setpointRad, double tolerance) {
    return Math.abs(getRotation().getRadians() - setpointRad) < tolerance;
  }

  public boolean atAngularSetpoint(double setpointRad) {
    return atAngularSetpoint(setpointRad, Constants.Drive.angularErrorToleranceDeg);
  }

  public boolean atAutoRotateSetpoint() {
    return atAngularSetpoint(targetAutoRotateAngle.getRadians());
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyroInputs.yawAngle, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          DrivetrainConstants.frontLeft.moduleLocationX,
          DrivetrainConstants.frontLeft.moduleLocationY),
      new Translation2d(
          DrivetrainConstants.frontRight.moduleLocationX,
          DrivetrainConstants.frontRight.moduleLocationY),
      new Translation2d(
          DrivetrainConstants.backLeft.moduleLocationX,
          DrivetrainConstants.backLeft.moduleLocationY),
      new Translation2d(
          DrivetrainConstants.backRight.moduleLocationX,
          DrivetrainConstants.backRight.moduleLocationY)
    };
  }
}
