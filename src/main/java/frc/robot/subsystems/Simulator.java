package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetectionIOSim;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {

  Scenario scenario = Scenario.THREE_CORAL;
  Side side = Side.LEFT;

  private enum Scenario {
    TELEOP,
    ONE_CORAL_TWO_ALGAE_CENTER,
    THREE_CORAL
  }

  private enum Side {
    LEFT,
    RIGHT
  }

  private enum SimulatedEventType {
    END_EFFECTOR_DETECT_CORAL,
    END_EFFECTOR_DETECT_ALGAE,
    END_EFFECTOR_NO_CORAL,
    END_EFFECTOR_NO_ALGAE,
    CORAL_IN_PICKUP_AREA,
    CORAL_NOT_IN_PICKUP_AREA,
    CORAL_IN_INDEXER,
    CORAL_NOT_IN_INDEXER,
    CORAL_VISIBLE,
    CORAL_NOT_VISIBLE
  }

  private enum EventStatus {
    ENABLED,
    DISABLED
  }

  private class SimulatedEvent {
    private double eventTime;
    private EventStatus eventStatus;
    private String eventName;
    private SimulatedEventType eventType;
    private Translation2d position;

    SimulatedEvent(
        double eventTime, String eventName, SimulatedEventType eventType, EventStatus eventStatus) {
      this(eventTime, eventName, eventType, null, eventStatus);
    }

    SimulatedEvent(
        double eventTime,
        String eventName,
        SimulatedEventType eventType,
        Translation2d position,
        EventStatus eventStatus) {
      this.eventTime = eventTime;
      this.eventName = eventName;
      this.eventType = eventType;
      this.eventStatus = eventStatus;
      this.position = position;
    }
  }

  private List<SimulatedEvent> buildScenario() {
    switch (scenario) {
      case ONE_CORAL_TWO_ALGAE_CENTER:
        return List.of(
            new SimulatedEvent(
                0.0, "Preload ready", SimulatedEventType.CORAL_IN_PICKUP_AREA, EventStatus.ENABLED),
            new SimulatedEvent(
                0.5,
                "Preload pickup",
                SimulatedEventType.END_EFFECTOR_DETECT_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                0.7,
                "Cradle empty",
                SimulatedEventType.CORAL_NOT_IN_PICKUP_AREA,
                EventStatus.ENABLED),
            new SimulatedEvent(
                1.4,
                "SAFE_DISTANCE drop",
                SimulatedEventType.END_EFFECTOR_NO_CORAL,
                EventStatus.DISABLED),
            new SimulatedEvent(
                1.6,
                "DRIVE_IN drop",
                SimulatedEventType.END_EFFECTOR_NO_CORAL,
                EventStatus.DISABLED),
            new SimulatedEvent(
                3.2, "Score coral", SimulatedEventType.END_EFFECTOR_NO_CORAL, EventStatus.ENABLED),
            new SimulatedEvent(
                4.7,
                "Pickup algae 1",
                SimulatedEventType.END_EFFECTOR_DETECT_ALGAE,
                EventStatus.ENABLED),
            new SimulatedEvent(
                8.0,
                "Score algae 1",
                SimulatedEventType.END_EFFECTOR_NO_ALGAE,
                EventStatus.ENABLED),
            new SimulatedEvent(
                11.3,
                "Pickup algae 2",
                SimulatedEventType.END_EFFECTOR_DETECT_ALGAE,
                EventStatus.ENABLED),
            new SimulatedEvent(
                15.1,
                "Score algae 2",
                SimulatedEventType.END_EFFECTOR_NO_ALGAE,
                EventStatus.ENABLED));

      case THREE_CORAL:
        return List.of(
            new SimulatedEvent(
                0.0, "Preload ready", SimulatedEventType.CORAL_IN_PICKUP_AREA, EventStatus.ENABLED),
            new SimulatedEvent(
                0.5,
                "Preload pickup",
                SimulatedEventType.END_EFFECTOR_DETECT_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                0.7,
                "Cradle empty",
                SimulatedEventType.CORAL_NOT_IN_PICKUP_AREA,
                EventStatus.ENABLED),
            new SimulatedEvent(
                3.9,
                "Score coral 1",
                SimulatedEventType.END_EFFECTOR_NO_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                5.4,
                "See coral 2",
                SimulatedEventType.CORAL_VISIBLE,
                new Translation2d(
                    Robot.alliance == Alliance.Blue ? 2.3 : 15.5,
                    (Robot.alliance == Alliance.Blue && side == Side.RIGHT)
                            || (Robot.alliance == Alliance.Red && side == Side.LEFT)
                        ? 1.63
                        : 6.5),
                EventStatus.ENABLED),
            new SimulatedEvent(
                6.5, "Coral 2 indexer", SimulatedEventType.CORAL_IN_INDEXER, EventStatus.ENABLED),
            new SimulatedEvent(
                6.6, "Coral 2 ready", SimulatedEventType.CORAL_IN_PICKUP_AREA, EventStatus.ENABLED),
            new SimulatedEvent(
                6.6, "Indexer clear", SimulatedEventType.CORAL_NOT_IN_INDEXER, EventStatus.ENABLED),
            new SimulatedEvent(
                6.8,
                "Coral 2 picked up",
                SimulatedEventType.END_EFFECTOR_DETECT_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                7.0,
                "Cradle empty",
                SimulatedEventType.CORAL_NOT_IN_PICKUP_AREA,
                EventStatus.ENABLED),
            new SimulatedEvent(
                9.55,
                "Score coral 2",
                SimulatedEventType.END_EFFECTOR_NO_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                9.75,
                "See coral 3",
                SimulatedEventType.CORAL_VISIBLE,
                new Translation2d(
                    Robot.alliance == Alliance.Blue ? 2.0 : 15.8,
                    (Robot.alliance == Alliance.Blue && side == Side.RIGHT)
                            || (Robot.alliance == Alliance.Red && side == Side.LEFT)
                        ? 1.9
                        : 6.3),
                EventStatus.ENABLED),
            new SimulatedEvent(
                12.3, "Coral 3 indexer", SimulatedEventType.CORAL_IN_INDEXER, EventStatus.ENABLED),
            new SimulatedEvent(
                12.4,
                "Coral 3 ready",
                SimulatedEventType.CORAL_IN_PICKUP_AREA,
                EventStatus.ENABLED),
            new SimulatedEvent(
                12.5,
                "Indexer clear",
                SimulatedEventType.CORAL_NOT_IN_INDEXER,
                EventStatus.ENABLED),
            new SimulatedEvent(
                12.6,
                "Coral 3 picked up",
                SimulatedEventType.END_EFFECTOR_DETECT_CORAL,
                EventStatus.ENABLED),
            new SimulatedEvent(
                12.8,
                "Cradle empty",
                SimulatedEventType.CORAL_NOT_IN_PICKUP_AREA,
                EventStatus.ENABLED),
            new SimulatedEvent(
                15.5,
                "Score coral 3",
                SimulatedEventType.END_EFFECTOR_NO_CORAL,
                EventStatus.ENABLED));

      default:
        return List.of();
    }
  }

  private List<SimulatedEvent> events;
  private Iterator<SimulatedEvent> iterator;
  private SimulatedEvent nextEvent;
  private Timer disabledTimer = new Timer();

  private EndEffectorIOSim endEffectorIOSim;
  private IndexerIOSim indexerIOSim;
  private VisionObjectDetectionIOSim visionObjectDetectionIOSim;

  public Simulator(
      EndEffectorIOSim endEffectorIOSim,
      IndexerIOSim indexerIOSim,
      VisionObjectDetectionIOSim visionObjectDetectionIOSim) {
    this.endEffectorIOSim = endEffectorIOSim;
    this.indexerIOSim = indexerIOSim;
    this.visionObjectDetectionIOSim = visionObjectDetectionIOSim;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Sim/MatchTime", Robot.matchTimer.get());
    if (DriverStation.isEnabled()) {
      if (events == null) {
        System.out.println("Select DISABLED for at least 2 aeconds before enabling!");
        System.exit(1);
      }
    } else if (DriverStation.isDSAttached()) {
      disabledTimer.start();
      if (disabledTimer.hasElapsed(2)) {
        events = buildScenario();
        iterator = events.iterator();
        if (iterator.hasNext()) {
          nextEvent = iterator.next();
        } else {
          nextEvent = null;
        }
      }
    }
    if (nextEvent != null) {
      if (Robot.matchTimer.get() >= nextEvent.eventTime) {
        if (nextEvent.eventStatus == EventStatus.ENABLED) {
          Logger.recordOutput("Sim/EventName", nextEvent.eventName);
          Logger.recordOutput("Sim/EventType", nextEvent.eventType);
          switch (nextEvent.eventType) {
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
                  nextEvent.position, WPIUtilJNI.now() * 1.0e-6);
              break;
            case CORAL_NOT_VISIBLE:
              visionObjectDetectionIOSim.noCoral();
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
  }
}
