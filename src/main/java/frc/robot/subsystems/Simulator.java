package frc.robot.subsystems;

import java.util.Iterator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutonomousSelector.AutoName;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetectionIOSim;

public class Simulator extends SubsystemBase {

  public static final AutoName simulatedAuto = AutoName.THREE_CORAL_RIGHT;
  private final Anomaly anomaly = Anomaly.NONE;
  private final TeleopScenario teleopScenario = TeleopScenario.LOOK_FROM_APRILTAG;

  private enum Anomaly {
    NONE,
    DROP_CORAL1_EARLY,
    DROP_CORAL1_LATE,
    DROP_CORAL2_LATE,
    DROP_ALGAE1_EARLY,
    RELEASE_TRIGGER_EARLY
  }

  private enum TeleopScenario {
    NONE,
    SCORE_L4,
    LOOK_FROM_APRILTAG
  }

  private enum EventType {
    SET_POSE,
    END_EFFECTOR_DETECT_CORAL,
    END_EFFECTOR_DETECT_ALGAE,
    END_EFFECTOR_NO_CORAL,
    END_EFFECTOR_NO_ALGAE,
    CORAL_IN_PICKUP_AREA,
    CORAL_NOT_IN_PICKUP_AREA,
    CORAL_IN_INDEXER,
    CORAL_NOT_IN_INDEXER,
    CORAL_VISIBLE,
    CORAL_NOT_VISIBLE,
    PRESS_A,
    HOLD_A,
    RELEASE_A,
    PRESS_B,
    HOLD_B,
    RELEASE_B,
    PRESS_X,
    HOLD_X,
    RELEASE_X,
    PRESS_Y,
    HOLD_Y,
    RELEASE_Y,
    PRESS_LEFT_BUMPER,
    HOLD_LEFT_BUMPER,
    RELEASE_LEFT_BUMPER,
    PRESS_RIGHT_BUMPER,
    HOLD_RIGHT_BUMPER,
    RELEASE_RIGHT_BUMPER,
    RELEASE_POV,
    PRESS_UP_POV,
    HOLD_UP_POV,
    PRESS_RIGHT_POV,
    HOLD_RIGHT_POV,
    PRESS_DOWN_POV,
    HOLD_DOWN_POV,
    PRESS_LEFT_POV,
    HOLD_LEFT_POV,
    PRESS_LEFT_TRIGGER,
    HOLD_LEFT_TRIGGER,
    RELEASE_LEFT_TRIGGER,
    PRESS_RIGHT_TRIGGER,
    HOLD_RIGHT_TRIGGER,
    RELEASE_RIGHT_TRIGGER,
    PRESS_LEFT_STICK,
    HOLD_LEFT_STICK,
    RELEASE_LEFT_STICK,
    PRESS_RIGHT_STICK,
    HOLD_RIGHT_STICK,
    RELEASE_RIGHT_STICK,
    MOVE_JOYSTICK_DRIVE,
    MOVE_JOYSTICK_TURN
  }

  private enum EventStatus {
    ACTIVE,
    INACTIVE
  }

  private enum POVDirection {
    NONE(-1),
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    public final int value;

    private POVDirection(int value) {
      this.value = value;
    }
  }

  private enum ControllerAxis {
    LEFT(2),
    RIGHT(3);

    public final int value;

    private ControllerAxis(int value) {
      this.value = value;
    }
  }

  private class SimEvent {
    private double eventTime;
    private EventStatus eventStatus;
    private String eventName;
    private EventType eventType;
    private Pose2d pose;

    SimEvent(double eventTime, String eventName, EventType eventType) {
      this(eventTime, eventName, eventType, EventStatus.ACTIVE);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, EventStatus eventStatus) {
      this(eventTime, eventName, eventType, (Translation2d) null, eventStatus);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, Translation2d position) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero));
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Translation2d position,
        EventStatus eventStatus) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero), eventStatus);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, Pose2d pose) {
      this(eventTime, eventName, eventType, pose, EventStatus.ACTIVE);
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Pose2d pose,
        EventStatus eventStatus) {
      this.eventTime = eventTime;
      this.eventName = eventName;
      this.eventType = eventType;
      this.eventStatus = eventStatus;
      this.pose = pose;
    }
  }

  private List<SimEvent> buildAutoScenario() {
    double t = 0;
    switch (simulatedAuto) {
      case ONE_CORAL_TWO_ALGAE_CENTER:
        return List.of(
            new SimEvent(t, "Preload ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.5, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(
                t += 0.7,
                "Early drop",
                EventType.END_EFFECTOR_NO_CORAL,
                anomaly == Anomaly.DROP_CORAL1_EARLY ? EventStatus.ACTIVE : EventStatus.INACTIVE),
            new SimEvent(
                t += 0.2,
                "Late drop",
                EventType.END_EFFECTOR_NO_CORAL,
                anomaly == Anomaly.DROP_CORAL1_LATE ? EventStatus.ACTIVE : EventStatus.INACTIVE),
            new SimEvent(t += 1.6, "Score coral", EventType.END_EFFECTOR_NO_CORAL),
            new SimEvent(t += 1.5, "Pickup algae 1", EventType.END_EFFECTOR_DETECT_ALGAE),
            new SimEvent(t += 3.3, "Score algae 1", EventType.END_EFFECTOR_NO_ALGAE),
            new SimEvent(t += 3.3, "Pickup algae 2", EventType.END_EFFECTOR_DETECT_ALGAE),
            new SimEvent(t += 3.8, "Score algae 2", EventType.END_EFFECTOR_NO_ALGAE));

      case THREE_CORAL_LEFT:
      case THREE_CORAL_RIGHT:
        return List.of(
            new SimEvent(t, "Preload ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.5, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 3.2, "Score coral 1", EventType.END_EFFECTOR_NO_CORAL),
            new SimEvent(
                t += 1.5,
                "See coral 2",
                EventType.CORAL_VISIBLE,
                new Translation2d(
                    Robot.alliance == Alliance.Blue ? 2.3 : 15.5,
                    (Robot.alliance == Alliance.Blue && simulatedAuto == AutoName.THREE_CORAL_RIGHT)
                            || (Robot.alliance == Alliance.Red
                                && simulatedAuto == AutoName.THREE_CORAL_LEFT)
                        ? 1.63
                        : 6.5)),
            new SimEvent(t += 1.1, "Coral 2 indexer", EventType.CORAL_IN_INDEXER),
            new SimEvent(t += 0.1, "Coral 2 ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.0, "Indexer clear", EventType.CORAL_NOT_IN_INDEXER),
            new SimEvent(t += 0.2, "Coral 2 picked up", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 2.55, "Score coral 2", EventType.END_EFFECTOR_NO_CORAL),
            new SimEvent(
                t += 0.2,
                "See coral 3",
                EventType.CORAL_VISIBLE,
                new Translation2d(
                    Robot.alliance == Alliance.Blue ? 2.0 : 15.8,
                    (Robot.alliance == Alliance.Blue && simulatedAuto == AutoName.THREE_CORAL_RIGHT)
                            || (Robot.alliance == Alliance.Red
                                && simulatedAuto == AutoName.THREE_CORAL_LEFT)
                        ? 1.9
                        : 6.3)),
            new SimEvent(t += 2.55, "Coral 3 indexer", EventType.CORAL_IN_INDEXER),
            new SimEvent(t += 0.1, "Coral 3 ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.1, "Indexer clear", EventType.CORAL_NOT_IN_INDEXER),
            new SimEvent(t += 0.1, "Coral 3 picked up", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 2.7, "Score coral 3", EventType.END_EFFECTOR_NO_CORAL));

      default:
        return List.of();
    }
  }

  private List<SimEvent> buildTeleopScenario() {
    double t = 0;
    switch (teleopScenario) {
      case SCORE_L4:
        return List.of(
            new SimEvent(
                t, "Start pose", EventType.SET_POSE, new Pose2d(15.0, 2.0, Rotation2d.k180deg)),
            new SimEvent(t += 0.5, "Deploy intake", EventType.PRESS_LEFT_POV),
            new SimEvent(t += 0.1, "Coral indexer", EventType.CORAL_IN_INDEXER),
            new SimEvent(t += 0.05, "Coral ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.05, "Indexer clear", EventType.CORAL_NOT_IN_INDEXER),
            new SimEvent(t += 0.2, "Coral picked up", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 0.1, "Retract intake", EventType.PRESS_LEFT_POV),
            new SimEvent(t += 0.1, "Drive to reef", EventType.HOLD_B),
            // new SimEvent(t += 3.0, "Score coral L4", EventType.HOLD_RIGHT_TRIGGER),
            new SimEvent(
                t += 0.7,
                "Set new Pose",
                EventType.SET_POSE,
                new Pose2d(13.474446, 3.3063179999999996, Rotation2d.k180deg),
                anomaly == Anomaly.RELEASE_TRIGGER_EARLY
                    ? EventStatus.ACTIVE
                    : EventStatus.INACTIVE),
            new SimEvent(
                t += 0.7,
                "Release Trigger Earily",
                EventType.RELEASE_B,
                anomaly == Anomaly.RELEASE_TRIGGER_EARLY
                    ? EventStatus.ACTIVE
                    : EventStatus.INACTIVE),
            new SimEvent(
                t += 0.7,
                "Set new Pose",
                EventType.SET_POSE,
                new Pose2d(13.474446, 3.3063179999999996, Rotation2d.k180deg),
                anomaly == Anomaly.RELEASE_TRIGGER_EARLY
                    ? EventStatus.ACTIVE
                    : EventStatus.INACTIVE),
            new SimEvent(t += 0.5, "Chain Algae", EventType.HOLD_Y),
            new SimEvent(t += 0.0, "Coral released", EventType.END_EFFECTOR_NO_CORAL),
            new SimEvent(
                t += 0.7,
                "Set new Pose",
                EventType.SET_POSE,
                new Pose2d(13.474446, 3.3063179999999996, Rotation2d.k180deg),
                anomaly == Anomaly.RELEASE_TRIGGER_EARLY
                    ? EventStatus.ACTIVE
                    : EventStatus.INACTIVE),
            new SimEvent(t += 0.1, "Release score trigger", EventType.RELEASE_RIGHT_TRIGGER),
            new SimEvent(
                t += 0.7,
                "Back away from reef",
                EventType.SET_POSE,
                new Pose2d(15.0, 2.0, Rotation2d.k180deg)),
            new SimEvent(t += 0.1, "Score complete", EventType.RELEASE_B));
      case LOOK_FROM_APRILTAG:
        return List.of(
            new SimEvent(
                t, "Start pose", EventType.SET_POSE, new Pose2d(15.0, 2.0, Rotation2d.k180deg)),
            new SimEvent(
                t += 1.0,
                "Look away from AprilTag",
                EventType.MOVE_JOYSTICK_DRIVE,
                new Pose2d(-1, 1, Rotation2d.k180deg)));
      default:
        return List.of();
    }
  }

  private List<SimEvent> autoEvents;
  private List<SimEvent> teleopEvents;
  private List<SimEvent> events;
  private Iterator<SimEvent> iterator;
  private SimEvent nextEvent;
  private final Timer disabledTimer = new Timer();
  private final Timer matchTimer = new Timer();
  XboxController hid = RobotContainer.driver.getHID(); // the real WPILib XboxController
  int hidPort = hid.getPort();
  int activeButtonBitmask;
  int momentaryButtonBitmask;
  boolean releasePOV;
  boolean releaseLeftTrigger;
  boolean releaseRightTrigger;

  private final Drive drive;
  private final EndEffectorIOSim endEffectorIOSim;
  private final IndexerIOSim indexerIOSim;
  private final VisionObjectDetectionIOSim visionObjectDetectionIOSim;
  private boolean currentModeAutonomous;

  public Simulator(
      Drive drive,
      EndEffectorIOSim endEffectorIOSim,
      IndexerIOSim indexerIOSim,
      VisionObjectDetectionIOSim visionObjectDetectionIOSim) {
    this.drive = drive;
    this.endEffectorIOSim = endEffectorIOSim;
    this.indexerIOSim = indexerIOSim;
    this.visionObjectDetectionIOSim = visionObjectDetectionIOSim;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Sim/MatchTime", matchTimer.get());
    if (!DriverStation.isEnabled()) {
      if (events != null) {
        events = null;
        nextEvent = null;
        disabledTimer.stop();
        disabledTimer.reset();
      }
      if (DriverStation.isDSAttached()) {
        disabledTimer.start();
      }
    } else {
      // can only release buttons when enabled
      releaseMomentaryButtons();
      if (releasePOV) {
        holdPOV(POVDirection.NONE);
        releasePOV = false;
      }
      if (releaseLeftTrigger) {
        releaseTrigger(ControllerAxis.LEFT);
        releaseLeftTrigger = false;
      }
      if (releaseRightTrigger) {
        releaseTrigger(ControllerAxis.RIGHT);
        releaseRightTrigger = false;
      }

      if (events == null) {
        if (!disabledTimer.hasElapsed(2)) {
          // wait for alliance color update
          System.out.println("Select DISABLED for at least 2 aeconds before enabling!");
          System.exit(1);
        }
        autoEvents = buildAutoScenario();
        teleopEvents = buildTeleopScenario();
      }
      if (events == null || DriverStation.isAutonomous() != currentModeAutonomous) {
        currentModeAutonomous = DriverStation.isAutonomous();
        events = currentModeAutonomous ? autoEvents : teleopEvents;
        matchTimer.restart();
        iterator = events.iterator();
        if (iterator.hasNext()) {
          nextEvent = iterator.next();
        } else {
          nextEvent = null;
        }
      }
    }

    while (nextEvent != null && matchTimer.get() >= nextEvent.eventTime) {
      if (nextEvent.eventStatus == EventStatus.ACTIVE) {
        Logger.recordOutput("Sim/EventName", nextEvent.eventName);
        Logger.recordOutput("Sim/EventType", nextEvent.eventType);
        switch (nextEvent.eventType) {
          case SET_POSE:
            drive.resetPose(nextEvent.pose);
            break;
          case END_EFFECTOR_NO_CORAL:
            endEffectorIOSim.simCoralReleased();
            break;
          case END_EFFECTOR_NO_ALGAE:
            endEffectorIOSim.simAlgaeReleased();
            break;
          case END_EFFECTOR_DETECT_CORAL:
            endEffectorIOSim.simCoralHeld();
            break;
          case END_EFFECTOR_DETECT_ALGAE:
            endEffectorIOSim.simAlgaeHeld();
            break;
          case CORAL_IN_PICKUP_AREA:
            indexerIOSim.simCoralDetectedInPickupArea();
            break;
          case CORAL_NOT_IN_PICKUP_AREA:
            indexerIOSim.simCoralNOTDetectedInPickupArea();
            break;
          case CORAL_IN_INDEXER:
            indexerIOSim.simCoralDetectedInIndexer();
            break;
          case CORAL_NOT_IN_INDEXER:
            indexerIOSim.simCoralNOTDetectedInIndexer();
            break;
          case CORAL_VISIBLE:
            visionObjectDetectionIOSim.coralDetected(
                nextEvent.pose.getTranslation(), WPIUtilJNI.now() * 1.0e-6);
            break;
          case CORAL_NOT_VISIBLE:
            visionObjectDetectionIOSim.noCoral();
            break;
          case PRESS_A:
            pressButton(XboxController.Button.kA);
            break;
          case HOLD_A:
            holdButton(XboxController.Button.kA);
            break;
          case RELEASE_A:
            releaseButton(XboxController.Button.kA);
            break;
          case PRESS_B:
            pressButton(XboxController.Button.kB);
            break;
          case HOLD_B:
            holdButton(XboxController.Button.kB);
            break;
          case RELEASE_B:
            releaseButton(XboxController.Button.kB);
            break;
          case PRESS_X:
            pressButton(XboxController.Button.kX);
            break;
          case HOLD_X:
            holdButton(XboxController.Button.kX);
            break;
          case RELEASE_X:
            releaseButton(XboxController.Button.kX);
            break;
          case PRESS_Y:
            pressButton(XboxController.Button.kY);
            break;
          case HOLD_Y:
            holdButton(XboxController.Button.kY);
            break;
          case RELEASE_Y:
            releaseButton(XboxController.Button.kY);
            break;
          case PRESS_LEFT_BUMPER:
            pressButton(XboxController.Button.kLeftBumper);
            break;
          case HOLD_LEFT_BUMPER:
            holdButton(XboxController.Button.kLeftBumper);
            break;
          case RELEASE_LEFT_BUMPER:
            releaseButton(XboxController.Button.kLeftBumper);
            break;
          case PRESS_RIGHT_BUMPER:
            pressButton(XboxController.Button.kRightBumper);
            break;
          case HOLD_RIGHT_BUMPER:
            holdButton(XboxController.Button.kRightBumper);
            break;
          case RELEASE_RIGHT_BUMPER:
            releaseButton(XboxController.Button.kRightBumper);
            break;
          case RELEASE_POV:
            holdPOV(POVDirection.NONE);
            break;
          case PRESS_UP_POV:
            pressPOV(POVDirection.UP);
            break;
          case HOLD_UP_POV:
            holdPOV(POVDirection.UP);
            break;
          case PRESS_RIGHT_POV:
            pressPOV(POVDirection.RIGHT);
            break;
          case HOLD_RIGHT_POV:
            holdPOV(POVDirection.RIGHT);
            break;
          case PRESS_DOWN_POV:
            pressPOV(POVDirection.DOWN);
            break;
          case HOLD_DOWN_POV:
            holdPOV(POVDirection.DOWN);
            break;
          case PRESS_LEFT_POV:
            pressPOV(POVDirection.LEFT);
            break;
          case HOLD_LEFT_POV:
            holdPOV(POVDirection.LEFT);
            break;
          case PRESS_LEFT_TRIGGER:
            pressTrigger(ControllerAxis.LEFT);
            break;
          case HOLD_LEFT_TRIGGER:
            holdTrigger(ControllerAxis.LEFT);
            break;
          case RELEASE_LEFT_TRIGGER:
            releaseTrigger(ControllerAxis.LEFT);
            break;
          case PRESS_RIGHT_TRIGGER:
            pressTrigger(ControllerAxis.RIGHT);
            break;
          case HOLD_RIGHT_TRIGGER:
            holdTrigger(ControllerAxis.RIGHT);
            break;
          case RELEASE_RIGHT_TRIGGER:
            releaseTrigger(ControllerAxis.RIGHT);
            break;
          case PRESS_LEFT_STICK:
            pressButton(XboxController.Button.kLeftStick);
            break;
          case HOLD_LEFT_STICK:
            holdButton(XboxController.Button.kLeftStick);
            break;
          case RELEASE_LEFT_STICK:
            releaseButton(XboxController.Button.kLeftStick);
            break;
          case PRESS_RIGHT_STICK:
            pressButton(XboxController.Button.kRightStick);
            break;
          case HOLD_RIGHT_STICK:
            holdButton(XboxController.Button.kRightStick);
            break;
          case RELEASE_RIGHT_STICK:
            releaseButton(XboxController.Button.kRightStick);
            break;
          case MOVE_JOYSTICK_DRIVE:
            moveJoystickLeft(nextEvent.pose.getX(), nextEvent.pose.getY());
            break;
          case MOVE_JOYSTICK_TURN:
            moveJoystickRight(nextEvent.pose.getX(), nextEvent.pose.getY());
            break;
        }
      }
      if (iterator.hasNext()) {
        nextEvent = iterator.next();
      } else {
        nextEvent = null;
      }
    }
  }

  private void holdButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
    DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
    DriverStationSim.notifyNewData();
  }

  private void releaseButton(XboxController.Button button) {
    activeButtonBitmask &= ~(1 << (button.value - 1));
    DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
    DriverStationSim.notifyNewData();
  }

  private void pressButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
    momentaryButtonBitmask |= 1 << (button.value - 1);
    DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
    DriverStationSim.notifyNewData();
  }

  private void releaseMomentaryButtons() {
    activeButtonBitmask &= ~momentaryButtonBitmask;
    momentaryButtonBitmask = 0;
    DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
    DriverStationSim.notifyNewData();
  }

  private void holdPOV(POVDirection direction) {
    DriverStationSim.setJoystickPOV(hidPort, 0, direction.value);
    DriverStationSim.notifyNewData();
  }

  private void moveJoystickLeft(double x, double y) {
    DriverStationSim.setJoystickAxis(hidPort, 0, -x); // Move X axis
    DriverStationSim.setJoystickAxis(hidPort, 1, -y); // Move Y axis
    DriverStationSim.notifyNewData();
  }

  private void moveJoystickRight(double x, double y) {
    DriverStationSim.setJoystickAxis(hidPort, 4, -x); // Move X axis
    DriverStationSim.setJoystickAxis(hidPort, 5, -y); // Move Y axis
    DriverStationSim.notifyNewData();
  }

  private void pressPOV(POVDirection direction) {
    holdPOV(direction);
    releasePOV = true;
  }

  private void holdTrigger(ControllerAxis axis) {
    DriverStationSim.setJoystickAxis(hidPort, axis.value, 1.0);
    DriverStationSim.notifyNewData();
  }

  private void pressTrigger(ControllerAxis axis) {
    holdTrigger(axis);
    if (axis == ControllerAxis.LEFT) {
      releaseLeftTrigger = true;
    } else {
      releaseRightTrigger = true;
    }
  }

  private void releaseTrigger(ControllerAxis axis) {
    DriverStationSim.setJoystickAxis(hidPort, axis.value, 0.0);
    DriverStationSim.notifyNewData();
  }
}
