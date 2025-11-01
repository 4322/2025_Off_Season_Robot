package frc.robot.commands;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LoggedTunableNumber;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;
  private final boolean highMode;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;
  private Pose2d lastTargetPose;
  private double pathDistance;
  private boolean reachedGoal;

  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/DriveKd");
  private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/ThetaKp");
  private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/ThetaKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveMaxDeceleration =
      new LoggedTunableNumber("DriveToPose/driveMaxDeceleration");
  private static final LoggedTunableNumber driveMaxDecelerationHigh =
      new LoggedTunableNumber("DriveToPose/driveMaxDecelerationL4");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("DriveToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber thetaToleranceSlow =
      new LoggedTunableNumber("DriveToPose/ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");

  static {
    driveKp.initDefault(Constants.AutoScoring.drivekP);
    driveKd.initDefault(Constants.AutoScoring.drivekD);
    thetaKp.initDefault(Constants.Drive.autoRotatekP);
    thetaKd.initDefault(Constants.Drive.autoRotatekD);

    driveMaxVelocity.initDefault(Constants.AutoScoring.driveMaxVelocity);
    driveMaxVelocitySlow.initDefault(Constants.AutoScoring.driveMaxVelocitySlow);
    driveMaxAcceleration.initDefault(Constants.AutoScoring.driveMaxAcceleration);
    driveMaxDeceleration.initDefault(Constants.AutoScoring.driveMaxDeceleration);
    driveMaxDecelerationHigh.initDefault(Constants.AutoScoring.driveMaxDecelerationHigh);

    thetaMaxVelocity.initDefault(Constants.AutoScoring.thetaMaxVelocity);
    thetaMaxVelocitySlow.initDefault(Constants.AutoScoring.thetaMaxVelocitySlow);
    thetaMaxAcceleration.initDefault(Constants.AutoScoring.thetaMaxAcceleration);

    driveTolerance.initDefault(Constants.AutoScoring.driveTolerance);
    driveToleranceSlow.initDefault(Constants.AutoScoring.driveToleranceSlow);

    thetaTolerance.initDefault(Constants.Drive.angularErrorToleranceDeg);
    thetaToleranceSlow.initDefault(Constants.Drive.angularErrorToleranceDeg);

    ffMinRadius.initDefault(Constants.AutoScoring.ffMinRadius);
    ffMaxRadius.initDefault(Constants.AutoScoring.ffMaxRadius);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Pose2d pose, boolean highMode) {
    this(drive, false, pose, highMode);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, boolean slowMode, Pose2d pose, boolean highMode) {
    this(drive, slowMode, () -> pose, highMode);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier, boolean highMode) {
    this(drive, false, poseSupplier, highMode);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(
      Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier, boolean highMode) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    this.highMode = highMode;
    addRequirements(drive);
    thetaController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    Pose2d currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    drive.getFieldRelativeSpeeds().vxMetersPerSecond,
                    drive.getFieldRelativeSpeeds().vyMetersPerSecond)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getDegrees(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
    pathDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    // use acceleration at start of new path
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
            driveMaxAcceleration.get()));
    lastTargetPose = poseSupplier.get();
    reachedGoal = false;
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveMaxDeceleration.hasChanged(hashCode())
        || driveMaxDecelerationHigh.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || driveToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || thetaToleranceSlow.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              driveMaxAcceleration.get()));
      driveController.setTolerance(slowMode ? driveToleranceSlow.get() : driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
              thetaMaxAcceleration.get()));
      thetaController.setTolerance(slowMode ? thetaToleranceSlow.get() : thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    if (currentDistance <= Math.min(pathDistance / 2, 0.75)) {
      // use deceleration after path mid-point
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              highMode ? driveMaxDecelerationHigh.get() : driveMaxDeceleration.get()));
    }
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;

    if (!targetPose.equals(lastTargetPose)) {
      pathDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
      // use acceleration at start of new path
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              highMode ? driveMaxDecelerationHigh.get() : driveMaxDeceleration.get()));
      lastTargetPose = targetPose;
      reachedGoal = false;
    } else {
      if (!reachedGoal && driveErrorAbs < driveController.getPositionTolerance()) {
        reachedGoal = true;
      }
    }

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (reachedGoal) {
      driveVelocityScalar = 0.0;
    }
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getDegrees());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/DriveToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("DriveToPose/DriveToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    reachedGoal = false;
    drive.runVelocity(new ChassisSpeeds(), true);
    Logger.recordOutput("DriveToPose/DriveToPoseSetpoint", new Pose2d() {});
    Logger.recordOutput("DriveToPose/DriveToPoseGoal", new Pose2d() {});
  }

  /** Checks if the robot is at the final pose. */
  public boolean atGoal() {
    return reachedGoal && thetaErrorAbs < thetaController.getPositionTolerance();
  }

  /** Checks if the robot pose is within the allowed drive tolerance. */
  public boolean withinTolerance(double driveTolerance) {
    return running && Math.abs(driveErrorAbs) < driveTolerance;
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}
