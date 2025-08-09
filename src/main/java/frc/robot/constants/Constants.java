package frc.robot.constants;

public class Constants {

  // TODO all of these are placeholder values
  public static class EndEffector {
    public static final int endEffectorMotorID = 0;
    public static final int endEffectorSensorID = 0;

    public static final double ALGAE_HOLD_VOLTS = 3.0;
    public static final double CORAL_HOLD_VOLTS = 3.0;

    public static final double ALGAE_INTAKE_VOLTS = 3.0;
    public static final double CORAL_INTAKE_VOLTS = 3.0;

    public static final double ALGAE_RELEASE_VOLTS = 3.0;
    public static final double CORAL_RELEASE_VOLTS = 3.0;

    public static final double CURRENT_DETECTION_THRESHOLD = 0.0;

    public static final double MOTOR_BUS_CURRENT_LIMIT = 0;
    public static final double MOTOR_BUS_CURRENT_LIMIT_TIME = 0;
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 0;

    public static final double SENSOR_GREEN_THRESHOLD = 0;
    public static final double SENSOR_BLUE_THRESHOLD = 0;
    public static final double SENSOR_RED_THRESHOLD = 0;
  }

  public static class Deployer {
    public static final int deployerMotorID = 1;
    public static final double DEPLOY_VOLTAGE = 3.0;
  }

  public static class Indexer {
    public static final int indexerMotorID = 2;
    public static final double FEED_VOLTAGE = 3.0;
    public static final double FEED_SLOW_VOLTAGE = 3.0;
    public static final double EJECT_VOLTAGE = 3.0;
    public static final double REJECT_VOLTAGE = 3.0;
    public static final double REJECT_SLOW_VOLTAGE = 3.0;
  }

  public static class Rollers {
    public static final int rollersMotorID = 3;

    public static final double FEED_VOLTAGE = 3.0;
    public static final double FEED_SLOW_VOLTAGE = 3.0;
    public static final double EJECT_VOLTAGE = 3.0;
    public static final double REJECT_VOLTAGE = 3.0;
    public static final double REJECT_SLOW_VOLTAGE = 3.0;
  }

  public static class IntakeSuperstructure {
    public static final double INDEXER_RETRACT_TIMEOUT_SECONDS = 3.0;
    public static final double PICKUP_AREA_RETRACT_TIMEOUT_SECONDS = 3.0;
  }
}
