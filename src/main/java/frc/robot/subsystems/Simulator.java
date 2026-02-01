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
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {
  private static final RegressTests regressTest = RegressTests.CONTROLLER_TEST;
  public static AutoName autoScenario;
  private TeleopScenario teleopScenario;
  private List<TeleAnomaly> teleAnomalies;
  private List<AutoAnomaly> autoAnomalies;
  private String currentScenario;
  private final Map<Integer, Double> axisValues = new HashMap<Integer, Double>();
  public static boolean slipwheel = false;

  private enum RegressTests {
    DOUBLE_DOUBLE,
    DOUBLE_AUTO,
    DOUBLE_TELEOP,
    APRIL_TAG_TEST,
    CONTROLLER_TEST
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
    DROP_ALGAE1_EARLY,
    RELEASE_TRIGGER_EARLY
  }

  private enum TeleopScenario {
    NONE,
    SCORE_L4,
    LOOK_FROM_APRILTAG_RED_SIDE,
    CONTROLLER_TEST1,
    CONTROLLER_TEST2
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
    MOVE_JOYSTICK_TURN,
    STOP_JOYSTICK,
    ENABLE_WHEEL_SLIP,
    DISABLE_WHEEL_SLIP,
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
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

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

      case APRIL_TAG_TEST -> List.of(
          new RegressionTest("Test 1", TeleopScenario.LOOK_FROM_APRILTAG_RED_SIDE, Alliance.Blue));

      case CONTROLLER_TEST -> List.of(
          new RegressionTest("Controller Test 1", TeleopScenario.CONTROLLER_TEST1, Alliance.Blue),
          new RegressionTest("Controller Test 2", TeleopScenario.CONTROLLER_TEST2, Alliance.Blue));

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
    int eventNum = 1;

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

      case LOOK_FROM_APRILTAG_RED_SIDE -> List.of(
          new SimEvent(t, "Enable wheel slip", EventType.ENABLE_WHEEL_SLIP),
          new SimEvent(
              t += 1.0, "Start pose", EventType.SET_POSE, new Pose2d(17, 4, Rotation2d.k180deg)),
          new SimEvent(
              t += 2.0,
              "Drive Away from still looking at",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, 0, Rotation2d.k180deg)),
          new SimEvent(t += 4, "Stop", EventType.STOP_JOYSTICK),
          new SimEvent(
              t += 2.0,
              "Turn away from AprilTag",
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, 0.5, Rotation2d.k180deg)),
          new SimEvent(t += 1.5, "Stop", EventType.STOP_JOYSTICK),
          new SimEvent(
              t += 2.0,
              "Drive to AprilTag Looking away",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.5, 0, Rotation2d.k180deg)),
          new SimEvent(t += 4, "Stop", EventType.STOP_JOYSTICK),
          new SimEvent(
              t += 2.0,
              "Turn away from AprilTag",
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, -0.5, Rotation2d.k180deg)),
          new SimEvent(t += 1.5, "Stop", EventType.STOP_JOYSTICK),
          new SimEvent(t += 4, "Disable wheel slip", EventType.DISABLE_WHEEL_SLIP));

      case CONTROLLER_TEST1 -> List.of(
          new SimEvent(
              t += 1.0, "Start pose", EventType.SET_POSE, new Pose2d(12, 4, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0.5, 0, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.STOP_JOYSTICK),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_UP_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, -0.5, Rotation2d.k180deg)));

      case CONTROLLER_TEST2 -> List.of(
          // check that controls were released from CONTROLLER_TEST1 when switching to this scenario
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, 0, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(-0.5, 0, Rotation2d.k180deg)));

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
  int currentPOV;

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
    Logger.recordOutput("Sim/MatchTime", matchTimer.get());

    DriverStationSim.setJoystickAxisCount(hidPort, 6);
    DriverStationSim.notifyNewData();

    // can only release buttons when enabled
    releaseMomentaryButtons();

    // refresh controller values that don't persist automatically
    for (Map.Entry<Integer, Double> entry : axisValues.entrySet()) {
      DriverStationSim.setJoystickAxis(hidPort, entry.getKey(), entry.getValue());
    }
    DriverStationSim.setJoystickPOV(hidPort, 0, currentPOV);
    DriverStationSim.notifyNewData();

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
          case PRESS_LEFT_TRIGGER -> pressTrigger(ControllerAxis.LEFT_TRIGGER);
          case HOLD_LEFT_TRIGGER -> holdTrigger(ControllerAxis.LEFT_TRIGGER);
          case RELEASE_LEFT_TRIGGER -> releaseTrigger(ControllerAxis.LEFT_TRIGGER);
          case PRESS_RIGHT_TRIGGER -> pressTrigger(ControllerAxis.RIGHT_TRIGGER);
          case HOLD_RIGHT_TRIGGER -> holdTrigger(ControllerAxis.RIGHT_TRIGGER);
          case RELEASE_RIGHT_TRIGGER -> releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
          case PRESS_LEFT_STICK -> pressButton(XboxController.Button.kLeftStick);
          case HOLD_LEFT_STICK -> holdButton(XboxController.Button.kLeftStick);
          case RELEASE_LEFT_STICK -> releaseButton(XboxController.Button.kLeftStick);
          case PRESS_RIGHT_STICK -> pressButton(XboxController.Button.kRightStick);
          case HOLD_RIGHT_STICK -> holdButton(XboxController.Button.kRightStick);
          case RELEASE_RIGHT_STICK -> releaseButton(XboxController.Button.kRightStick);
          case MOVE_JOYSTICK_DRIVE -> {
            persistAxis(ControllerAxis.LEFT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.LEFT_Y, -currentEvent.pose.getX());
          }
          case MOVE_JOYSTICK_TURN -> {
            persistAxis(ControllerAxis.RIGHT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.RIGHT_Y, -currentEvent.pose.getX());
          }
          case STOP_JOYSTICK -> stopJoystick();
          case ENABLE_WHEEL_SLIP -> slipwheel = true;
          case DISABLE_WHEEL_SLIP -> slipwheel = false;
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
    DriverStationSim.resetData(); // goes to disconnected state
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(events == autoEvents);
    eventIterator = events.iterator();

    // reset all controls
    holdPOV(POVDirection.NONE);
    releaseTrigger(ControllerAxis.LEFT_TRIGGER);
    releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
    stopJoystick();
    disabledTimer.start();
    activeButtonBitmask = 0;
    momentaryButtonBitmask = 0;
    DriverStationSim.setJoystickButtons(hidPort, 0);

    if (currentAlliance == Alliance.Red) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
    } else {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
    }
    // update controls and change from disconnected to disabled so alliance color can update
    DriverStationSim.notifyNewData();

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
    currentPOV = direction.value;
  }

  private void pressPOV(POVDirection direction) {
    holdPOV(direction);
    currentPOV = POVDirection.NONE.value; // cancel refresh
  }

  public void persistAxis(ControllerAxis axis, double value) {
    DriverStationSim.setJoystickAxis(hidPort, axis.value, value);
    DriverStationSim.notifyNewData();
    axisValues.put(axis.value, value);
  }

  private void stopJoystick() {
    persistAxis(ControllerAxis.LEFT_X, 0.0);
    persistAxis(ControllerAxis.LEFT_Y, 0.0);
    persistAxis(ControllerAxis.RIGHT_X, 0.0);
    persistAxis(ControllerAxis.RIGHT_Y, 0.0);
  }

  private void holdTrigger(ControllerAxis axis) {
    persistAxis(axis, 1.0);
  }

  private void releaseTrigger(ControllerAxis axis) {
    persistAxis(axis, 0.0);
  }

  private void pressTrigger(ControllerAxis axis) {
    persistAxis(axis, 1.0);
    axisValues.put(axis.value, 0.0); // cancel refresh
  }

  public static boolean wheelSlip() {
    return slipwheel;
  }

  public static AutoName getAutoScenario() {
    return autoScenario;
  }
}
