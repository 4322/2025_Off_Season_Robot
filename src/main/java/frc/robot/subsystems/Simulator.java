package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
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
    RELEASE_B
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
            new SimEvent(t += 0.5, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
            new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
            new SimEvent(t += 0.0, "Drive to reef", EventType.HOLD_B),
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
  XboxController hid = RobotContainer.driver.getHID(); // the real WPILib XboxController
  SimDeviceSim xboxSim = new SimDeviceSim("Xbox[" + hid.getPort() + "]");
  // Button index mapping:
  // A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6, Back = 7, Start = 8, L-stick = 9, R-stick = 10

  private final Drive drive;
  private final EndEffectorIOSim endEffectorIOSim;
  private final IndexerIOSim indexerIOSim;
  private final VisionObjectDetectionIOSim visionObjectDetectionIOSim;

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
    Logger.recordOutput("Sim/MatchTime", Robot.matchTimer.get());
    if (DriverStation.isDSAttached()) {
      disabledTimer.start();
    }
    if (DriverStation.isEnabled()) {
      if (!disabledTimer.hasElapsed(2)) {
        System.out.println("Select DISABLED for at least 2 aeconds before enabling!");
        System.exit(1);
      }
      if (events == null) {
        autoEvents = buildAutoScenario();
        teleopEvents = buildTeleopScenario();
        events = DriverStation.isAutonomous() ? autoEvents : teleopEvents;
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
              xboxSim.getBoolean("Button 2").set(true);
              break;
            case RELEASE_B:
              xboxSim.getBoolean("Button 2").set(false);
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
