// package frc.robot.subsystems.elevator;

// import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
// import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
// import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
// import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
// import com.reduxrobotics.motorcontrol.nitrate.settings.FramePeriodSettings;
// import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
// import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
// import com.reduxrobotics.motorcontrol.nitrate.types.EnabledDebugFrames;
// import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
// import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
// import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
// import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
// import com.reduxrobotics.motorcontrol.requests.FollowMotorRequest;
// import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.constants.Constants;

// public class ElevatorIONitrate implements ElevatorIO {
//   private final Nitrate leaderMotor;
//   private final Nitrate followerMotor;
//   private double lastRequestedPosMeters;
//   private double lastRequestedPosRotations;
//   private PIDPositionRequest elvPositionRequest =
//       new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
//   private PIDPositionRequest elvSlowPositionRequest =
//       new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);
//   private FollowMotorRequest followerRequest;

//   public ElevatorIONitrate() {
//     // Initialize leader and follower motors
//     leaderMotor = new Nitrate(Constants.Elevator.frontMotorID, MotorType.kCu60);
//     followerMotor = new Nitrate(Constants.Elevator.backMotorID, MotorType.kCu60);

//     // Setup config objects
//     NitrateSettings frontConfig = new NitrateSettings();
//     NitrateSettings backConfig = new NitrateSettings();

//     followerRequest = new FollowMotorRequest(leaderMotor);

//     frontConfig.setPIDSettings(
//         PIDSettings.defaultSettings(PIDConfigSlot.kSlot0)
//             .setPID(
//                 Constants.Elevator.fast_kP, Constants.Elevator.fast_kI,
// Constants.Elevator.fast_kD)
//             .setGravitationalFeedforward(Constants.Elevator.kG)
//             .setMinwrapConfig(new MinwrapConfig.Disabled())
//             .setMotionProfileAccelLimit(
//                 metersToRotations(Constants.Elevator.fastAccelerationMetersPerSec2))
//             .setMotionProfileDeaccelLimit(
//                 metersToRotations(Constants.Elevator.fastDecelerationMetersPerSec2))
//             .setMotionProfileVelocityLimit(
//                 metersToRotations(Constants.Elevator.fastVelocityMetersPerSec))
//             .setISaturation(Constants.Elevator.iSat)
//             .setIZone(Constants.Elevator.iZone)
//             .setRampLimit(240),
//         PIDConfigSlot.kSlot0);

//     frontConfig.setPIDSettings(
//         PIDSettings.defaultSettings(PIDConfigSlot.kSlot1)
//             .setPID(Constants.Elevator.slow_kP, 0, 0)
//             .setGravitationalFeedforward(Constants.Elevator.kG)
//             .setMinwrapConfig(new MinwrapConfig.Disabled())
//             .setMotionProfileAccelLimit(
//                 metersToRotations(Constants.Elevator.slowAccelerationMetersPerSec2))
//             .setMotionProfileDeaccelLimit(
//                 metersToRotations(Constants.Elevator.slowDecelerationMetersPerSec2))
//             .setMotionProfileVelocityLimit(
//                 metersToRotations(Constants.Elevator.slowVelocityMetersPerSec))
//             .setISaturation(Constants.Elevator.iSat)
//             .setIZone(Constants.Elevator.iZone)
//             .setRampLimit(240),
//         PIDConfigSlot.kSlot1);

//     frontConfig.setOutputSettings(
//         OutputSettings.defaultSettings()
//             .setIdleMode(Constants.Elevator.motorIdleMode)
//             .setInvert(Constants.Elevator.motorFrontInvert));

//     frontConfig.setElectricalLimitSettings(
//         ElectricalLimitSettings.defaultSettings()
//             .setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps)
//             .setStatorCurrentLimit(Constants.Elevator.statorCurrentLimitAmps));

//     frontConfig.setFeedbackSensorSettings(FeedbackSensorSettings.defaultSettings());

//     frontConfig.setFramePeriodSettings(
//         FramePeriodSettings.defaultSettings()
//             .setEnabledPIDDebugFrames(
//                 new EnabledDebugFrames()
//                     .setKgControlEffort(Constants.debugPIDModeEnabled)
//                     .setKpControlEffort(Constants.debugPIDModeEnabled)
//                     .setKiControlEffort(Constants.debugPIDModeEnabled)
//                     .setFeedbackError((Constants.debugPIDModeEnabled))
//                     .setTotalControlEffort(Constants.debugPIDModeEnabled)));

//     backConfig.setOutputSettings(
//         OutputSettings.defaultSettings()
//             .setIdleMode(Constants.Elevator.motorIdleMode)
//             .setInvert(Constants.Elevator.motorBackInvert));

//     backConfig.setElectricalLimitSettings(
//         ElectricalLimitSettings.defaultSettings()
//             .setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps)
//             .setStatorCurrentLimit(Constants.Elevator.statorCurrentLimitAmps));

//     backConfig.setFeedbackSensorSettings(FeedbackSensorSettings.defaultSettings());

//     followerRequest.setInverted(false);

//     NitrateSettings leaderConfigStatus = leaderMotor.setSettings(frontConfig, 0.1, 5);
//     NitrateSettings followerConfigStatus = followerMotor.setSettings(backConfig, 0.1, 5);
//     followerMotor.setRequest(followerRequest);
//     // get position is an internal encoder, so we need to set it
//     // 6 to 1 gear ratio for elevator first stage
//     // 9 to 1 gear ratio for elevator second stage

//     if (!leaderConfigStatus.isEmpty()) {
//       DriverStation.reportError(
//           "Nitrate "
//               + leaderMotor.getAddress().getDeviceId()
//               + " (leader motor) failed to configure",
//           false);
//     }
//     if (!followerConfigStatus.isEmpty()) {
//       DriverStation.reportError(
//           "Nitrate "
//               + followerMotor.getAddress().getDeviceId()
//               + " (follower motor) failed to configure",
//           false);
//     }
//   }

//   @Override
//   public void updateInputs(ElevatorIOInputs inputs) {
//     // Implementation for updating inputs from the Nitrate hardware
//     inputs.leaderConnected = leaderMotor.isConnected();
//     inputs.followerConnected = followerMotor.isConnected();

//     inputs.requestedPosMeters = lastRequestedPosMeters;
//     inputs.requestedPosRotations = lastRequestedPosRotations;

//     inputs.leaderheightMeters = rotationsToMeters(leaderMotor.getPosition());
//     inputs.followerHeightMeters = rotationsToMeters(followerMotor.getPosition());

//     inputs.followerVelocityMetersPerSecond = followerMotor.getVelocity();
//     inputs.velMetersPerSecond = leaderMotor.getVelocity();

//     inputs.leaderSupplyAmps = leaderMotor.getBusCurrent();
//     inputs.followerSupplyAmps = followerMotor.getBusCurrent();

//     inputs.leaderStatorAmps = leaderMotor.getStatorCurrent();
//     inputs.followerStatorAmps = followerMotor.getStatorCurrent();

//     inputs.leadertempCelcius = leaderMotor.getMotorTemperatureFrame().getValue();
//     inputs.followertempCelcius = followerMotor.getMotorTemperatureFrame().getValue();

//     inputs.leaderControllerTempCelcius = leaderMotor.getControllerTemperatureFrame().getValue();
//     inputs.followerControllerTempCelcius =
// followerMotor.getControllerTemperatureFrame().getValue();

//     inputs.leaderVoltage = leaderMotor.getAppliedVoltageFrame().getValue();
//     inputs.leaderEncoderRotations = leaderMotor.getPosition();

//     inputs.followerVoltage = followerMotor.getAppliedVoltageFrame().getValue();
//     inputs.followerEncoderRotations = followerMotor.getPosition();

//     if (Constants.debugPIDModeEnabled) {
//       inputs.kPeffort = leaderMotor.getPIDDebugFrames().kPControlEffortFrame.getValue();
//       inputs.kIeffort = leaderMotor.getPIDDebugFrames().kIControlEffortFrame.getValue();
//       inputs.kGeffort = leaderMotor.getPIDDebugFrames().kGControlEffortFrame.getValue();
//       inputs.totalEffort = leaderMotor.getPIDDebugFrames().totalControlEffortFrame.getValue();
//       inputs.feedbackError = leaderMotor.getPIDDebugFrames().feedbackErrorFrame.getValue();
//     }
//   }

//   @Override
//   public void setPosition(double elevatorPositionMeters) {
//     lastRequestedPosRotations = metersToRotations(elevatorPositionMeters);
//     leaderMotor.setPosition(lastRequestedPosRotations);
//     followerMotor.setPosition(lastRequestedPosRotations);
//     stop(IdleMode.kBrake);
//   }

//   @Override
//   public void requestSlowHeightMeters(double heightMeters) {
//     followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
//     lastRequestedPosMeters = heightMeters;
//     lastRequestedPosRotations = metersToRotations(heightMeters);
//     leaderMotor.setRequest(elvSlowPositionRequest.setPosition(lastRequestedPosRotations));
//   }

//   @Override
//   public void requestHeightMeters(double heightMeters) {
//     followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
//     lastRequestedPosMeters = heightMeters;
//     lastRequestedPosRotations = metersToRotations(heightMeters);
//     leaderMotor.setRequest(elvPositionRequest.setPosition(lastRequestedPosRotations));
//   }

//   @Override
//   public void setVoltage(double voltage) {
//     followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
//     leaderMotor.setVoltage(voltage);
//     lastRequestedPosMeters = -1;
//     lastRequestedPosRotations = -1;
//   }

//   @Override
//   public void stop(IdleMode idleMode) {
//     followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
//     leaderMotor.stop(idleMode);
//     leaderMotor.setVoltage(0); // work around stop not working
//     lastRequestedPosMeters = -1;
//     lastRequestedPosRotations = -1;
//   }

//   @Override
//   public Nitrate getNitrate() {
//     followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
//     return leaderMotor;
//   }

//   public double metersToRotations(double meters) {
//     return meters
//         / (Math.PI * Constants.Elevator.beltPulleyPitchDiameterMeters)
//         * Constants.Elevator.gearRatio;
//   }

//   public double rotationsToMeters(double rotations) {
//     return rotations
//         * Math.PI
//         * Constants.Elevator.beltPulleyPitchDiameterMeters
//         / Constants.Elevator.gearRatio;
//   }
// }
