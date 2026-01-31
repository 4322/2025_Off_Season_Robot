package frc.robot.subsystems;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutonomousSelector.AutoName;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetectionIOSim;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {
  private static final RegressTests regressTest = RegressTests.DOUBLE_DOUBLE;
  public static AutoName autoScenario;
  private TeleopScenario teleopScenario;
  private List<TeleAnomaly> teleAnomalies;
  private List<AutoAnomaly> autoAnomalies;
  private String currentScenario;

  private enum RegressTests {
    DOUBLE_DOUBLE,
    DOUBLE_AUTO,
    DOUBLE_TELEOP
  }

  private enum TeleAnomaly {
    NONE,
    DROP_CORAL1_EARLY,
    DROP_CORAL1_LATE,
    DROP_CORAL2_LATE,
    DROP_ALGAE1_EARLY
  }

  private enum AutoAnomaly {
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
    END_OF_SCENARIO
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

  private static class RegressionTest {
    private String name;
    private AutoName autoScenario;
    private List<AutoAnomaly> autoAnomalies;
    private TeleopScenario teleopScenario;
    private List<TeleAnomaly> teleAnomalies;
    private Alliance alliance;

    RegressionTest(
        String name,
        AutoName autoScenario,
        List<AutoAnomaly> autoAnomalies,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this.name = name;
      this.autoScenario = autoScenario;
      this.autoAnomalies = autoAnomalies;
      this.teleopScenario = teleopScenario;
      this.teleAnomalies = teleAnomalies;
      this.alliance = alliance;
    }

    RegressionTest(
        String name, AutoName autoScenario, List<AutoAnomaly> autoAnomalies, Alliance alliance) {
      this(name, autoScenario, autoAnomalies, null, null, alliance);
    }

    RegressionTest(
        String name,
        AutoName autoScenario,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, autoScenario, (List<AutoAnomaly>) null, teleopScenario, teleAnomalies, alliance);
    }

    RegressionTest(
        String name, AutoName autoScenario, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, autoScenario, teleopScenario, null, alliance);
    }

    RegressionTest(String name, AutoName autoScenario, Alliance alliance) {
      this(name, autoScenario, null, null, alliance);
    }

    RegressionTest(
        String name,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, null, teleopScenario, teleAnomalies, alliance);
    }

    RegressionTest(String name, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, teleopScenario, null, alliance);
    }
  }

  private List<RegressionTest> regressionTestCases() {
    return switch (regressTest) {
      case DOUBLE_DOUBLE -> List.of(
          new RegressionTest(
              "Test 1",
              AutoName.ONE_CORAL_TWO_ALGAE_CENTER,
              List.of(AutoAnomaly.NONE),
              TeleopScenario.SCORE_L4,
              List.of(TeleAnomaly.NONE),
              Alliance.Blue),
          new RegressionTest(
              "Test 2", AutoName.THREE_CORAL_RIGHT, TeleopScenario.SCORE_L4, Alliance.Blue));
      case DOUBLE_AUTO -> List.of(
          new RegressionTest("Test 1", AutoName.ONE_CORAL_TWO_ALGAE_CENTER, Alliance.Blue),
          new RegressionTest(
              "Test 2", AutoName.THREE_CORAL_RIGHT, List.of(AutoAnomaly.NONE), Alliance.Red));
      case DOUBLE_TELEOP -> List.of(
          new RegressionTest("Test 1", TeleopScenario.SCORE_L4, Alliance.Blue),
          new RegressionTest(
              "Test 2", TeleopScenario.SCORE_L4, List.of(TeleAnomaly.NONE), Alliance.Red));
      default -> List.of();
    };
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
    if (autoScenario == null) {
      return List.of();
    }
    double t = 0.0;
    return switch (autoScenario) {
      case ONE_CORAL_TWO_ALGAE_CENTER -> List.of(
          new SimEvent(t, "Preload ready", EventType.CORAL_IN_PICKUP_AREA),
          new SimEvent(t += 0.5, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
          new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
          new SimEvent(
              t += 0.7,
              "Early drop",
              EventType.END_EFFECTOR_NO_CORAL,
              aListContains(AutoAnomaly.DROP_CORAL1_EARLY)
                  ? EventStatus.ACTIVE
                  : EventStatus.INACTIVE),
          new SimEvent(
              t += 0.2,
              "Late drop",
              EventType.END_EFFECTOR_NO_CORAL,
              aListContains(AutoAnomaly.DROP_CORAL1_LATE)
                  ? EventStatus.ACTIVE
                  : EventStatus.INACTIVE),
          new SimEvent(t += 1.6, "Score coral", EventType.END_EFFECTOR_NO_CORAL),
          new SimEvent(t += 1.5, "Pickup algae 1", EventType.END_EFFECTOR_DETECT_ALGAE),
          new SimEvent(t += 3.3, "Score algae 1", EventType.END_EFFECTOR_NO_ALGAE),
          new SimEvent(t += 3.3, "Pickup algae 2", EventType.END_EFFECTOR_DETECT_ALGAE),
          new SimEvent(t += 3.8, "Score algae 2", EventType.END_EFFECTOR_NO_ALGAE),
          new SimEvent(t += 3.0, "Final Movement", EventType.END_OF_SCENARIO));

      case THREE_CORAL_LEFT, THREE_CORAL_RIGHT -> List.of(
          new SimEvent(t, "Preload ready", EventType.CORAL_IN_PICKUP_AREA),
          new SimEvent(t += 0.5, "Preload pickup", EventType.END_EFFECTOR_DETECT_CORAL),
          new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
          new SimEvent(t += 3.2, "Score coral 1", EventType.END_EFFECTOR_NO_CORAL),
          new SimEvent(
              t += 1.5,
              "See coral 2",
              EventType.CORAL_VISIBLE,
              new Translation2d(
                  currentAlliance == Alliance.Blue ? 2.3 : 15.5,
                  (currentAlliance == Alliance.Blue && autoScenario == AutoName.THREE_CORAL_RIGHT)
                          || (currentAlliance == Alliance.Red
                              && autoScenario == AutoName.THREE_CORAL_LEFT)
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
                  currentAlliance == Alliance.Blue ? 2.0 : 15.8,
                  (currentAlliance == Alliance.Blue && autoScenario == AutoName.THREE_CORAL_RIGHT)
                          || (currentAlliance == Alliance.Red
                              && autoScenario == AutoName.THREE_CORAL_LEFT)
                      ? 1.9
                      : 6.3)),
          new SimEvent(t += 2.55, "Coral 3 indexer", EventType.CORAL_IN_INDEXER),
          new SimEvent(t += 0.1, "Coral 3 ready", EventType.CORAL_IN_PICKUP_AREA),
          new SimEvent(t += 0.1, "Indexer clear", EventType.CORAL_NOT_IN_INDEXER),
          new SimEvent(t += 0.1, "Coral 3 picked up", EventType.END_EFFECTOR_DETECT_CORAL),
          new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
          new SimEvent(t += 2.7, "Score coral 3", EventType.END_EFFECTOR_NO_CORAL),
          new SimEvent(t += 3.0, "Final Movement", EventType.END_OF_SCENARIO));

      default -> List.of();
    };
  }

  private List<SimEvent> buildTeleopScenario() {
    if (teleopScenario == null) {
      return List.of();
    }
    double t = 0.0;
    return switch (teleopScenario) {
      case SCORE_L4 -> List.of(
          new SimEvent(
              t,
              "Start pose",
              EventType.SET_POSE,
              currentAlliance == Alliance.Blue
                  ? new Pose2d(2.0, 6.0, Rotation2d.kZero)
                  : new Pose2d(15.0, 2.0, Rotation2d.k180deg)),
          new SimEvent(t += 0.5, "Deploy intake", EventType.PRESS_LEFT_POV),
          new SimEvent(t += 0.1, "Coral indexer", EventType.CORAL_IN_INDEXER),
          new SimEvent(t += 0.05, "Coral ready", EventType.CORAL_IN_PICKUP_AREA),
          new SimEvent(t += 0.05, "Indexer clear", EventType.CORAL_NOT_IN_INDEXER),
          new SimEvent(t += 0.2, "Coral picked up", EventType.END_EFFECTOR_DETECT_CORAL),
          new SimEvent(t += 0.2, "Cradle empty", EventType.CORAL_NOT_IN_PICKUP_AREA),
          new SimEvent(t += 0.1, "Retract intake", EventType.PRESS_LEFT_POV),
          new SimEvent(t += 0.1, "Drive to reef", EventType.HOLD_B),
          new SimEvent(t += 3.0, "Score coral L4", EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 0.1, "Coral released", EventType.END_EFFECTOR_NO_CORAL),
          new SimEvent(t += 0.1, "Release score trigger", EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(t += 0.1, "Score complete", EventType.RELEASE_B),
          new SimEvent(t += 3.0, "Final Movement", EventType.END_OF_SCENARIO));
      default -> List.of();
    };
  }

  private boolean aListContains(AutoAnomaly aAnomaly) {
    if (autoAnomalies == null) {
      return false;
    }
    return autoAnomalies.contains(aAnomaly);
  }

  private boolean tListContains(TeleAnomaly tAnomaly) {
    if (teleAnomalies == null) {
      return false;
    }
    return teleAnomalies.contains(tAnomaly);
  }

  private Iterator<RegressionTest> regressionTestIterator;
  private RegressionTest currentRegressionTest;
  private List<SimEvent> autoEvents;
  private List<SimEvent> teleopEvents;
  private List<SimEvent> events;
  private Iterator<SimEvent> eventIterator;
  private SimEvent currentEvent;
  private Alliance currentAlliance;
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

  public Simulator(
      Drive drive,
      EndEffectorIOSim endEffectorIOSim,
      IndexerIOSim indexerIOSim,
      VisionObjectDetectionIOSim visionObjectDetectionIOSim) {
    this.drive = drive;
    this.endEffectorIOSim = endEffectorIOSim;
    this.indexerIOSim = indexerIOSim;
    this.visionObjectDetectionIOSim = visionObjectDetectionIOSim;
    regressionTestIterator = regressionTestCases().iterator();
    setNextRegressTest();
  }

  @Override
  public void periodic() {
    if (disabledTimer.isRunning()) {
      if (!disabledTimer.hasElapsed(2)) {
        return;
      }
      disabledTimer.stop();
      disabledTimer.reset();
      DriverStationSim.setEnabled(true);
      matchTimer.restart();
      currentEvent = eventIterator.next();
    }

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

    Logger.recordOutput("Sim/RegressionTest", currentRegressionTest.name);
    Logger.recordOutput("Sim/Alliance", currentAlliance.toString());
    Logger.recordOutput("Sim/Scenario", currentScenario);
    Logger.recordOutput("Sim/MatchTime", matchTimer.get());

    while (currentEvent != null && matchTimer.get() >= currentEvent.eventTime) {
      if (currentEvent.eventStatus == EventStatus.ACTIVE) {
        Logger.recordOutput("Sim/EventName", currentEvent.eventName);
        Logger.recordOutput("Sim/EventType", currentEvent.eventType);
        switch (currentEvent.eventType) {
          case SET_POSE -> drive.resetPose(currentEvent.pose);
          case END_EFFECTOR_NO_CORAL -> endEffectorIOSim.simCoralReleased();
          case END_EFFECTOR_NO_ALGAE -> endEffectorIOSim.simAlgaeReleased();
          case END_EFFECTOR_DETECT_CORAL -> endEffectorIOSim.simCoralHeld();
          case END_EFFECTOR_DETECT_ALGAE -> endEffectorIOSim.simAlgaeHeld();
          case CORAL_IN_PICKUP_AREA -> indexerIOSim.simCoralDetectedInPickupArea();
          case CORAL_NOT_IN_PICKUP_AREA -> indexerIOSim.simCoralNOTDetectedInPickupArea();
          case CORAL_IN_INDEXER -> indexerIOSim.simCoralDetectedInIndexer();
          case CORAL_NOT_IN_INDEXER -> indexerIOSim.simCoralNOTDetectedInIndexer();
          case CORAL_VISIBLE -> visionObjectDetectionIOSim.coralDetected(
              currentEvent.pose.getTranslation(), WPIUtilJNI.now() * 1.0e-6);
          case CORAL_NOT_VISIBLE -> visionObjectDetectionIOSim.noCoral();
          case PRESS_A -> pressButton(XboxController.Button.kA);
          case HOLD_A -> holdButton(XboxController.Button.kA);
          case RELEASE_A -> releaseButton(XboxController.Button.kA);
          case PRESS_B -> pressButton(XboxController.Button.kB);
          case HOLD_B -> holdButton(XboxController.Button.kB);
          case RELEASE_B -> releaseButton(XboxController.Button.kB);
          case PRESS_X -> pressButton(XboxController.Button.kX);
          case HOLD_X -> holdButton(XboxController.Button.kX);
          case RELEASE_X -> releaseButton(XboxController.Button.kX);
          case PRESS_Y -> pressButton(XboxController.Button.kY);
          case HOLD_Y -> holdButton(XboxController.Button.kY);
          case RELEASE_Y -> releaseButton(XboxController.Button.kY);
          case PRESS_LEFT_BUMPER -> pressButton(XboxController.Button.kLeftBumper);
          case HOLD_LEFT_BUMPER -> holdButton(XboxController.Button.kLeftBumper);
          case RELEASE_LEFT_BUMPER -> releaseButton(XboxController.Button.kLeftBumper);
          case PRESS_RIGHT_BUMPER -> pressButton(XboxController.Button.kRightBumper);
          case HOLD_RIGHT_BUMPER -> holdButton(XboxController.Button.kRightBumper);
          case RELEASE_RIGHT_BUMPER -> releaseButton(XboxController.Button.kRightBumper);
          case RELEASE_POV -> holdPOV(POVDirection.NONE);
          case PRESS_UP_POV -> pressPOV(POVDirection.UP);
          case HOLD_UP_POV -> holdPOV(POVDirection.UP);
          case PRESS_RIGHT_POV -> pressPOV(POVDirection.RIGHT);
          case HOLD_RIGHT_POV -> holdPOV(POVDirection.RIGHT);
          case PRESS_DOWN_POV -> pressPOV(POVDirection.DOWN);
          case HOLD_DOWN_POV -> holdPOV(POVDirection.DOWN);
          case PRESS_LEFT_POV -> pressPOV(POVDirection.LEFT);
          case HOLD_LEFT_POV -> holdPOV(POVDirection.LEFT);
          case PRESS_LEFT_TRIGGER -> pressTrigger(ControllerAxis.LEFT);
          case HOLD_LEFT_TRIGGER -> holdTrigger(ControllerAxis.LEFT);
          case RELEASE_LEFT_TRIGGER -> releaseTrigger(ControllerAxis.LEFT);
          case PRESS_RIGHT_TRIGGER -> pressTrigger(ControllerAxis.RIGHT);
          case HOLD_RIGHT_TRIGGER -> holdTrigger(ControllerAxis.RIGHT);
          case RELEASE_RIGHT_TRIGGER -> releaseTrigger(ControllerAxis.RIGHT);
          case PRESS_LEFT_STICK -> pressButton(XboxController.Button.kLeftStick);
          case HOLD_LEFT_STICK -> holdButton(XboxController.Button.kLeftStick);
          case RELEASE_LEFT_STICK -> releaseButton(XboxController.Button.kLeftStick);
          case PRESS_RIGHT_STICK -> pressButton(XboxController.Button.kRightStick);
          case HOLD_RIGHT_STICK -> holdButton(XboxController.Button.kRightStick);
          case RELEASE_RIGHT_STICK -> releaseButton(XboxController.Button.kRightStick);
          case END_OF_SCENARIO -> {}
        }
      }

      if (eventIterator.hasNext()) {
        currentEvent = eventIterator.next();
      } else {
        // end of scenario
        currentEvent = null;
        if (events == autoEvents && !teleopEvents.isEmpty()) {
          events = teleopEvents;
          resetScenario();
        } else {
          setNextRegressTest();
        }
      }
    }
  }

  private void resetScenario() {
    DriverStationSim.resetData();
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(events == autoEvents);
    eventIterator = events.iterator();
    releasePOV = false;
    releaseLeftTrigger = false;
    releaseRightTrigger = false;
    disabledTimer.start();
    if (currentAlliance == Alliance.Red) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
    } else {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
    }
    if (events == autoEvents) {
      currentScenario = autoScenario.toString();
    } else {
      currentScenario = teleopScenario.toString();
    }
  }

  private void setNextRegressTest() {
    if (!regressionTestIterator.hasNext()) {
      // All tests complete
      DriverStationSim.setEnabled(false); // exit cleanly
      System.exit(0);
    }
    currentRegressionTest = regressionTestIterator.next();
    autoScenario = currentRegressionTest.autoScenario;
    autoAnomalies = currentRegressionTest.autoAnomalies;
    teleopScenario = currentRegressionTest.teleopScenario;
    teleAnomalies = currentRegressionTest.teleAnomalies;
    currentAlliance = currentRegressionTest.alliance;
    autoEvents = buildAutoScenario();
    teleopEvents = buildTeleopScenario();
    if (!autoEvents.isEmpty()) {
      events = autoEvents;
    } else {
      events = teleopEvents;
    }
    resetScenario();
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

  public static AutoName getAutoScenario() {
    return autoScenario;
  }
}
