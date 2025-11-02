package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetectionIOSim;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {

  private EndEffectorIOSim endEffectorIOSim;
  private IndexerIOSim indexerIOSim;
  private VisionObjectDetectionIOSim visionObjectDetectionIOSim;

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

  private class SimulatedEvent {
    private double eventTime;
    private SimulatedEventType eventType;
    private Translation2d position;

    SimulatedEvent(double eventTime, SimulatedEventType eventType) {
      this(eventTime, eventType, null);
    }

    SimulatedEvent(double eventTime, SimulatedEventType eventType, Translation2d position) {
      this.eventTime = eventTime;
      this.eventType = eventType;
      this.position = position;
    }
  }

  private SimulatedEvent[] auto1Coral2AlgaeCenter = {
    // drop coral while driving in for 1 coral, 2 algae center auto
    new SimulatedEvent(0.0, SimulatedEventType.CORAL_IN_PICKUP_AREA),
    new SimulatedEvent(0.5, SimulatedEventType.END_EFFECTOR_DETECT_CORAL),
    new SimulatedEvent(0.7, SimulatedEventType.CORAL_NOT_IN_PICKUP_AREA),
    // new SimulatedEvent(1.4, SimulatedEventType.END_EFFECTOR_NO_CORAL), // drop before DRIVE_IN
    // new SimulatedEvent(1.6, SimulatedEventType.END_EFFECTOR_NO_CORAL), // drop during DRIVE_IN
    new SimulatedEvent(3.2, SimulatedEventType.END_EFFECTOR_NO_CORAL),
    new SimulatedEvent(4.7, SimulatedEventType.END_EFFECTOR_DETECT_ALGAE),
    new SimulatedEvent(8, SimulatedEventType.END_EFFECTOR_NO_ALGAE),
    new SimulatedEvent(11.3, SimulatedEventType.END_EFFECTOR_DETECT_ALGAE),
    new SimulatedEvent(15.1, SimulatedEventType.END_EFFECTOR_NO_ALGAE)
  };

  int nextEventIdx = 0;

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
    if (nextEventIdx < auto1Coral2AlgaeCenter.length) {
      if (Robot.matchTimer.get() >= auto1Coral2AlgaeCenter[nextEventIdx].eventTime) {
        Logger.recordOutput("Sim/NextEventType", auto1Coral2AlgaeCenter[nextEventIdx].eventType);
        switch (auto1Coral2AlgaeCenter[nextEventIdx].eventType) {
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
            break;
          case CORAL_NOT_VISIBLE:
            break;
        }
        nextEventIdx++;
      }
    }
  }
}
