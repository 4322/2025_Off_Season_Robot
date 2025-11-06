package frc.robot.subsystems;

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
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {

  public static final AutoName simulatedAuto = AutoName.ONE_CORAL_TWO_ALGAE_CENTER;
  private final Anomaly anomaly = Anomaly.DROP_CORAL1_LATE;
  private final TeleopScenario teleopScenario = TeleopScenario.SCORE_L4;

  private enum Anomaly {
    NONE,
    DROP_CORAL1_EARLY,
    DROP_CORAL1_LATE,
    DROP_CORAL2_LATE,
    DROP_ALGAE1_EARLY
  }

  private enum TeleopScenario {
    NONE,
    SCORE_L4
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
    HOLD_B,
    RELEASE_B,
    PRESS_LEFT_POV
  }

  private enum EventStatus {
    ACTIVE,
    INACTIVE
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
            new SimEvent(t, "Preload ready", EventType.CORAL_IN_PICKUP_AREA),
            new SimEvent(t += 0.3, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 0.1, "Drive to reef", EventType.HOLD_B),
            new SimEvent(t += 1.6, "Score coral", EventType.END_EFFECTOR_NO_CORAL),
            new SimEvent(t += 0.0, "Score complete", EventType.RELEASE_B));

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

    releaseMomentaryButtons();
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
          case HOLD_B:
            holdButton(XboxController.Button.kB);
            break;
          case RELEASE_B:
            releaseButton(XboxController.Button.kB);
          case PRESS_LEFT_POV:
            // TODO
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
}
