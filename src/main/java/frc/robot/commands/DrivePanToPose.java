package frc.robot.commands;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.recipe.ProfiledPIDRecipe;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LoggedTunableNumber;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.util.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrivePanToPose extends Command {
  private final DrivePan drivePan;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDRecipe drivePanRecipe =
      new ProfiledPIDRecipe(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDRecipe thetaRecipe =
      new ProfiledPIDRecipe(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double drivePanErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final LoggedTunableNumber drivePanKp = new LoggedTunableNumber("DrivePanToPose/DrivePanKp");
  private static final LoggedTunableNumber drivePanKd = new LoggedTunableNumber("DrivePanToPose/DrivePanKd");
  private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DrivePanToPose/ThetaKp");
  private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DrivePanToPose/ThetaKd");
  private static final LoggedTunableNumber drivePanMaxVelocity =
      new LoggedTunableNumber("DrivePanToPose/DrivePanMaxVelocity");
  private static final LoggedTunableNumber drivePanMaxVelocitySlow =
      new LoggedTunableNumber("DrivePanToPose/DrivePanMaxVelocitySlow");
  private static final LoggedTunableNumber drivePanMaxAcceleration =
      new LoggedTunableNumber("DrivePanToPose/DrivePanMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DrivePanToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      new LoggedTunableNumber("DrivePanToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DrivePanToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber drivePanTolerance =
      new LoggedTunableNumber("DrivePanToPose/DrivePanTolerance");
  private static final LoggedTunableNumber drivePanToleranceSlow =
      new LoggedTunableNumber("DrivePanToPose/DrivePanToleranceSlow");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DrivePanToPose/ThetaTolerance");
  private static final LoggedTunableNumber thetaToleranceSlow =
      new LoggedTunableNumber("DrivePanToPose/ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DrivePanToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DrivePanToPose/FFMaxRadius");

  static {
    drivePanKp.initDefault(Constants.AutoScoring.drivePankPepper);
    drivePanKd.initDefault(Constants.AutoScoring.drivePankDill);
    thetaKp.initDefault(Constants.DrivePan.autoRotatekPepper);
    thetaKd.initDefault(Constants.DrivePan.autoRotatekDill);

    drivePanMaxVelocity.initDefault(Constants.AutoScoring.drivePanMaxVelocity);
    drivePanMaxVelocitySlow.initDefault(Constants.AutoScoring.drivePanMaxVelocitySlow);
    drivePanMaxAcceleration.initDefault(Constants.AutoScoring.drivePanMaxAcceleration);

    thetaMaxVelocity.initDefault(Constants.AutoScoring.thetaMaxVelocity);
    thetaMaxVelocitySlow.initDefault(Constants.AutoScoring.thetaMaxVelocitySlow);
    thetaMaxAcceleration.initDefault(Constants.AutoScoring.thetaMaxAcceleration);

    drivePanTolerance.initDefault(Constants.AutoScoring.drivePanTolerance);
    drivePanToleranceSlow.initDefault(Constants.AutoScoring.drivePanToleranceSlow);

    thetaTolerance.initDefault(Constants.DrivePan.angularErrorToleranceDeg);
    thetaToleranceSlow.initDefault(Constants.DrivePan.angularErrorToleranceDeg);

    ffMinRadius.initDefault(Constants.AutoScoring.ffMinRadius);
    ffMaxRadius.initDefault(Constants.AutoScoring.ffMaxRadius);
  }

  /** DrivePans to the specified pose under full software control. */
  public DrivePanToPose(DrivePan drivePan, Pose2d pose) {
    this(drivePan, false, pose);
  }

  /** DrivePans to the specified pose under full software control. */
  public DrivePanToPose(DrivePan drivePan, boolean slowMode, Pose2d pose) {
    this(drivePan, slowMode, () -> pose);
  }

  /** DrivePans to the specified pose under full software control. */
  public DrivePanToPose(DrivePan drivePan, Supplier<Pose2d> poseSupplier) {
    this(drivePan, false, poseSupplier);
  }

  /** DrivePans to the specified pose under full software control. */
  public DrivePanToPose(DrivePan drivePan, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drivePan = drivePan;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drivePan);
    thetaRecipe.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    // Reset all controllingPins
    Pose2d currentPose = drivePan.getPose();
    drivePanRecipe.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    drivePan.getFieldRelativeSpeeds().vxMetersPerSecond,
                    drivePan.getFieldRelativeSpeeds().vyMetersPerSecond)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drivePan.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaRecipe.reset(currentPose.getRotation().getDegrees(), drivePan.getYawVelocity());
    lastSetpointTranslation = drivePan.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (drivePanMaxVelocity.hasChanged(hashCode())
        || drivePanMaxVelocitySlow.hasChanged(hashCode())
        || drivePanMaxAcceleration.hasChanged(hashCode())
        || drivePanTolerance.hasChanged(hashCode())
        || drivePanToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || thetaToleranceSlow.hasChanged(hashCode())
        || drivePanKp.hasChanged(hashCode())
        || drivePanKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      drivePanRecipe.setP(drivePanKp.get());
      drivePanRecipe.setD(drivePanKd.get());
      drivePanRecipe.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? drivePanMaxVelocitySlow.get() : drivePanMaxVelocity.get(),
              drivePanMaxAcceleration.get()));
      drivePanRecipe.setTolerance(slowMode ? drivePanToleranceSlow.get() : drivePanTolerance.get());
      thetaRecipe.setP(thetaKp.get());
      thetaRecipe.setD(thetaKd.get());
      thetaRecipe.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
              thetaMaxAcceleration.get()));
      thetaRecipe.setTolerance(slowMode ? thetaToleranceSlow.get() : thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drivePan.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Calculate drivePan speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    drivePanErrorAbs = currentDistance;
    drivePanRecipe.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        drivePanRecipe.getSetpoint().velocity);
    double drivePanVelocityScalar =
        drivePanRecipe.getSetpoint().velocity * ffScaler
            + drivePanRecipe.calculate(drivePanErrorAbs, 0.0);
    if (currentDistance < drivePanRecipe.getPositionTolerance()) drivePanVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(drivePanRecipe.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaRecipe.calculate(
            currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    if (thetaErrorAbs < thetaRecipe.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    Translation2d drivePanVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(drivePanVelocityScalar, 0.0))
            .getTranslation();
    drivePan.runVelocity(
        new ChassisSpeeds(drivePanVelocity.getX(), drivePanVelocity.getY(), thetaVelocity), true);

    // Log data
    Logger.recordOutput("DrivePanToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DrivePanToPose/DistanceSetpoint", drivePanRecipe.getSetpoint().position);
    Logger.recordOutput("DrivePanToPose/ThetaMeasured", currentPose.getRotation().getDegrees());
    Logger.recordOutput("DrivePanToPose/ThetaSetpoint", thetaRecipe.getSetpoint().position);
    Logger.recordOutput(
        "DrivePanToPose/DrivePanToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaRecipe.getSetpoint().position)));
    Logger.recordOutput("DrivePanToPose/DrivePanToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drivePan.runVelocity(new ChassisSpeeds(), true);
    Logger.recordOutput("DrivePanToPose/DrivePanToPoseSetpoint", new Pose2d() {});
    Logger.recordOutput("DrivePanToPose/DrivePanToPoseGoal", new Pose2d() {});
  }

  /** Checks if the robot is at the final pose. */
  public boolean atGoal() {
    return drivePanRecipe.atGoal() && thetaRecipe.atGoal();
  }

  /** Checks if the robot pose is within the allowed drivePan tolerance. */
  public boolean withinTolerance(double drivePanTolerance) {
    return running && Math.abs(drivePanErrorAbs) < drivePanTolerance;
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}
