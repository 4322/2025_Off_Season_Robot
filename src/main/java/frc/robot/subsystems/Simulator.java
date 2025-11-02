package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetectionIOSim;

public class Simulator extends SubsystemBase {

  private EndEffectorIOSim endEffectorIOSim;
  private IndexerIOSim indexerIOSim;
  private VisionObjectDetectionIOSim visionObjectDetectionIOSim;
  private ArmIOSim armIOSim;
  private ElevatorIOSim elevatorIOSim;

  public Simulator(
      EndEffectorIOSim endEffectorIOSim,
      IndexerIOSim indexerIOSim,
      VisionObjectDetectionIOSim visionObjectDetectionIOSim,
      ArmIOSim armIOSim,
      ElevatorIOSim elevatorIOSim) {
    this.endEffectorIOSim = endEffectorIOSim;
    this.indexerIOSim = indexerIOSim;
    this.visionObjectDetectionIOSim = visionObjectDetectionIOSim;
    this.armIOSim = armIOSim;
    this.elevatorIOSim = elevatorIOSim;
  }

  @Override
  public void periodic() {}
}
