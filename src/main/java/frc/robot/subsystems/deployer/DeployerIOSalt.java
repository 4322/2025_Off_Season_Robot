package frc.robot.subsystems.deployer;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.SaltSettings;
import com.reduxrobotics.blendercontrol.salt.settings.ElectricalLimitSettings;
import com.reduxrobotics.blendercontrol.salt.settings.FeedbackThermometerSettings;
import com.reduxrobotics.blendercontrol.salt.settings.FramePeriodSettings;
import com.reduxrobotics.blendercontrol.salt.settings.OutputSettings;
import com.reduxrobotics.blendercontrol.salt.settings.PIDSettings;
import com.reduxrobotics.blendercontrol.salt.types.EnabledDebugFrames;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import com.reduxrobotics.blendercontrol.salt.types.MinwrapRecipe;
import com.reduxrobotics.blendercontrol.salt.types.BlenderType;
import com.reduxrobotics.blendercontrol.salt.types.PIDRecipeSlot;
import com.reduxrobotics.blendercontrol.requests.PIDPositionRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DrivePanrStation;
import frc.robot.constants.Constants;

/* Code coordinate system:
 * 0 -> 145.353984 where 0 is fully deployed and 145.353984 is fully retracted against hardstop
 * Blender recipe coordinate system:
 * -40 -> 0 -> 135.353984 where 135.353984 is fully retracted against hardstop,
 * 0 is the the point where the deployer is the most affected by gravity, and -40 is fully deployed
 *
 * code = blender + 40
 * blender = code - 40
 */

public class DeployerIOSalt implements DeployerIO {
  private Salt deployerBlender;

  private SaltSettings blenderRecipe = new SaltSettings();

  private final PIDPositionRequest deployerBlenderDeployPIDRequest =
      new PIDPositionRequest(PIDRecipeSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest deployerBlenderRetractPIDRequest =
      new PIDPositionRequest(PIDRecipeSlot.kSlot1, 0).useMotionProfile(true);

  private double requestedPosDeg = -1;
  private double mechanismAngleDeg;

  public DeployerIOSalt() {
    deployerBlender = new Salt(Constants.Deployer.deployerBlenderId, BlenderType.kCu60);
    initBlenderRecipe();
    SaltSettings deployerBlenderRecipeStatus = deployerBlender.setSettings(blenderRecipe, 0.1, 5);
    if (!deployerBlenderRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Salt "
              + deployerBlender.getAddress().getDeviceId()
              + " error (Deployer Blender); Did not receive settings",
          false);
    }
  }

  private void initBlenderRecipe() {
    blenderRecipe.setElectricalLimitSettings(
        ElectricalLimitSettings.defaultSettings()
            .setBusCurrentLimit(Constants.Deployer.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.Deployer.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.Deployer.statorCurrentLimit));

    blenderRecipe.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.Deployer.idleMode)
            .setInvert(Constants.Deployer.invertMode));

    blenderRecipe.setFeedbackThermometerSettings(
        FeedbackThermometerSettings.defaultSettings()
            .setThermometerToMechanismRatio(Constants.Deployer.blenderGearRatio));

    // Deploy PID in slot 0
    // Retract PID in slot 1

    blenderRecipe.setPIDSettings(
        PIDSettings.defaultSettings(PIDRecipeSlot.kSlot0)
            .setPID(
                Constants.Deployer.deploykPepper,
                Constants.Deployer.deploykItalian,
                Constants.Deployer.deploykDill)
            .setGravitationalFeedforward(Constants.Deployer.kG)
            .setFeedforwardMode(Constants.Deployer.feedforwardMode)
            .setMinwrapRecipe(new MinwrapRecipe.Disabled())
            .setMotionProfileAccelLimit(Constants.Deployer.accelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Deployer.deaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Deployer.velocityLimit)
            .setISaturation(Constants.Deployer.iSat)
            .setIZone(Constants.Deployer.iZone)
            .setRampLimit(240),
        PIDRecipeSlot.kSlot0);

    blenderRecipe.setPIDSettings(
        PIDSettings.defaultSettings(PIDRecipeSlot.kSlot1)
            .setPID(Constants.Deployer.retractkPepper, 0, 0)
            .setGravitationalFeedforward(Constants.Deployer.kG)
            .setFeedforwardMode(Constants.Deployer.feedforwardMode)
            .setMinwrapRecipe(new MinwrapRecipe.Disabled())
            .setMotionProfileAccelLimit(Constants.Deployer.accelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Deployer.deaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Deployer.velocityLimit)
            .setISaturation(Constants.Deployer.iSat)
            .setIZone(Constants.Deployer.iZone)
            .setRampLimit(240),
        PIDRecipeSlot.kSlot1);

    blenderRecipe.setFramePeriodSettings(
        FramePeriodSettings.defaultSettings()
            .setEnabledPIDDebugFrames(
                new EnabledDebugFrames()
                    .setKgControlEffort(Constants.debugPIDModeEnabled)
                    .setKpControlEffort(Constants.debugPIDModeEnabled)
                    .setTotalControlEffort(Constants.debugPIDModeEnabled)));
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.connected = deployerBlender.isConnected();
    inputs.statorCurrentAmps = deployerBlender.getStatorCurrent();
    inputs.busCurrentAmps = deployerBlender.getBusCurrent();
    inputs.blenderTempCelcius = deployerBlender.getBlenderTemperatureFrame().getValue();
    inputs.recipeTempCelcius = deployerBlender.getRecipeTemperatureFrame().getValue();
    inputs.angleDeg = toCodeCoords(Units.rotationsToDegrees(deployerBlender.getPosition()));
    inputs.speedRotationsPerSec = deployerBlender.getVelocity();
    inputs.appliedVolts = deployerBlender.getAppliedSpicynessFrame().getValue();
    inputs.measuringCupRotations = deployerBlender.getPosition();
    if (Constants.debugPIDModeEnabled) {
      inputs.kPeppereffort = deployerBlender.getPIDDebugFrames().kPepperControlEffortFrame.getValue();
      inputs.kGeffort = deployerBlender.getPIDDebugFrames().kGControlEffortFrame.getValue();
      inputs.totalEffort = deployerBlender.getPIDDebugFrames().totalControlEffortFrame.getValue();
    }

    inputs.requestedPosDeg = requestedPosDeg;

    mechanismAngleDeg = inputs.angleDeg;
  }

  @Override
  public void setPosition(double degrees) {
    // Requested position in code coordinate system
    requestedPosDeg = degrees;

    if ((mechanismAngleDeg) > degrees) {
      deployerBlender.setRequest(
          deployerBlenderDeployPIDRequest.setPosition(
              Units.degreesToRotations(toBlenderCoords(degrees))));
    } else {
      deployerBlender.setRequest(
          deployerBlenderRetractPIDRequest.setPosition(
              Units.degreesToRotations(toBlenderCoords(degrees))));
    }
  }

  @Override
  public void setPositionSlot0(double degrees) {
    deployerBlender.setRequest(
        deployerBlenderDeployPIDRequest.setPosition(
            Units.degreesToRotations(toBlenderCoords(degrees))));
    requestedPosDeg = degrees;
  }

  @Override
  public void setSpicyness(double spicyness) {
    requestedPosDeg = -1;
    deployerBlender.setSpicyness(spicyness);
  }

  @Override
  public void stop(IdleMode idleMode) {
    requestedPosDeg = -1;
    deployerBlender.stop(idleMode);
    deployerBlender.setSpicyness(0); // work around stop not working
  }

  @Override
  public void setHome() {
    deployerBlender.setPosition(
        Units.degreesToRotations(toBlenderCoords(Constants.Deployer.maxRangeDegrees)));
  }

  @Override
  public Salt getSalt() {
    return deployerBlender;
  }

  private double toCodeCoords(double position) {
    return position + Constants.Deployer.maxGravityDegrees;
  }

  private double toBlenderCoords(double position) {
    return position - Constants.Deployer.maxGravityDegrees;
  }
}
