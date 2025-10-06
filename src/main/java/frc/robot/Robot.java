package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.reduxrobotics.canand.MessageLogger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public static Alliance alliance = DriverStation.Alliance.Blue;
  private Timer allianceUpdateTimer = new Timer();
  private DigitalInput homeButton = new DigitalInput(Constants.dioHomeButton);
  private Timer homeButtonTimer = new Timer();
  private DigitalInput coastButton = new DigitalInput(Constants.dioCoastButton);
  private Timer coastButtonTimer = new Timer();

  public static PathPlannerPath Leave;

  public static PathPlannerPath ThreeCoralStartToJuliet;
  public static PathPlannerPath JulietToFeed;
  public static PathPlannerPath KiloToFeed;
  public static PathPlannerPath FeedToKilo;
  public static PathPlannerPath FeedToLima;
  

  // Mirrors of the above
  public static PathPlannerPath ThreeCoralStartToEcho;
  public static PathPlannerPath EchoToFeed;
  public static PathPlannerPath FeedToDelta;
  public static PathPlannerPath DeltaToFeed;
  public static PathPlannerPath FeedToCharlie;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentMode) {
      case REAL:
        var directory = new File(Constants.logPath);
        if (!directory.exists()) {
          directory.mkdir();
        }
        var files = directory.listFiles();

        // delete all garbage hoot files and wpilogs not connected to ds before good wpilogs
        if (files != null) {
          for (File file : files) {
            if (file.getName().endsWith(".hoot")
                || (!file.getName().contains("-") && file.getName().endsWith(".wpilog"))) {
              file.delete();
              DriverStation.reportWarning("Deleted " + file.getName() + " to free up space", false);
            }
          }
        }

        // ensure that there is enough space on the roboRIO to log data
        if (directory.getFreeSpace() < Constants.minFreeSpace) {
          files = directory.listFiles();
          if (files != null) {
            // Sorting the files by name will ensure that the oldest files are deleted first
            files = Arrays.stream(files).sorted().toArray(File[]::new);

            long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

            // delete advantage kit logs
            for (File file : files) {
              if (file.getName().endsWith(".wpilog")) {
                try {
                  bytesToDelete -= Files.size(file.toPath());
                } catch (IOException e) {
                  DriverStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DriverStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DriverStation.reportError("Failed to delete " + file.getName(), false);
                }
                if (bytesToDelete <= 0) {
                  break;
                }
              }
            }
          }
        }

        // delete Redux logs if we still don't have enough space
        if (directory.getFreeSpace() < Constants.minFreeSpace) {
          files = directory.listFiles();
          if (files != null) {
            // Sorting the files by name will ensure that the oldest files are deleted first
            files = Arrays.stream(files).sorted().toArray(File[]::new);

            long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

            for (File file : files) {
              if (file.getName().endsWith(".rdxlog")) {
                try {
                  bytesToDelete -= Files.size(file.toPath());
                } catch (IOException e) {
                  DriverStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DriverStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DriverStation.reportError("Failed to delete " + file.getName(), false);
                }
                if (bytesToDelete <= 0) {
                  break;
                }
              }
            }
          }
        }

        Logger.addDataReceiver(
            new WPILOGWriter(Constants.logPath)); // Log to a USB stick is ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        RobotController.setBrownoutVoltage(Constants.brownoutVoltage);
        MessageLogger.openLog(Constants.logPath);

        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    Logger.start();
    Logger.disableConsoleCapture();

    try {
      Leave = PathPlannerPath.fromPathFile("Leave");

      ThreeCoralStartToJuliet = PathPlannerPath.fromPathFile("Three Coral Start To Juliet");
      JulietToFeed = PathPlannerPath.fromPathFile("Juliet To Feed");
      FeedToKilo = PathPlannerPath.fromPathFile("Feed To Kilo");
      KiloToFeed = PathPlannerPath.fromPathFile("Kilo To Feed");
      FeedToLima = PathPlannerPath.fromPathFile("Feed To Lima");
      

      ThreeCoralStartToEcho = PathPlannerPath.fromPathFile("Three Coral Start To Juliet").mirrorPath();
      EchoToFeed = PathPlannerPath.fromPathFile("Juliet To Feed").mirrorPath();
      FeedToDelta = PathPlannerPath.fromPathFile("Feed To Kilo").mirrorPath();
      DeltaToFeed = PathPlannerPath.fromPathFile("Kilo To Feed").mirrorPath();
      FeedToCharlie = PathPlannerPath.fromPathFile("Feed To Lima").mirrorPath();
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner paths", true);
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.configureAutonomousSelector();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    allianceUpdateTimer.start();

    if (Constants.currentMode == Constants.RobotMode.SIM) {
      // enable subsystems in sim mode
      RobotContainer.getSuperstructure().homeButtonActivated();
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    /* roboRIO settings to optimize Java memory use:
      echo "vm.overcommit_memory=1" >> /etc/sysctl.conf
      echo "vm.vfs_cache_pressure=1000" >> /etc/sysctl.conf
      echo "vm.swappiness=100" >> /etc/sysctl.conf
      sync
      power cycle the RIO

      To restiore default settings, edit /etc/sysctl.conf to set the
      following values:
        vm.overcommit_memory=2
        vm.vfs_cache_pressure=100
        vm.swappiness=60
        power cycle the RIO

      To stop the web server to save memory:
      /etc/init.d/systemWebServer stop; update-rc.d -f systemWebServer remove; sync
      chmod a-x /usr/local/natinst/etc/init.d/systemWebServer; sync

      To restart the web server in order to image the RIO:
      chmod a+x /usr/local/natinst/etc/init.d/systemWebServer; sync
      power cycle the RIO
    */

    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    if (allianceUpdateTimer.hasElapsed(1)) {
      Optional<Alliance> allianceOptional = DriverStation.getAlliance();
      if (allianceOptional.isPresent()) {
        alliance = allianceOptional.get();
      }
      allianceUpdateTimer.restart();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // close previous Redux log and open a new unique one
    MessageLogger.openLog(Constants.logPath);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    if (!homeButton.get() && Constants.currentMode != RobotMode.SIM) {
      homeButtonTimer.start();
      // button is pressed in
      if (homeButtonTimer.hasElapsed(Constants.homeButtonDelaySec)) {
        RobotContainer.getSuperstructure().homeButtonActivated();
        homeButtonTimer.stop();
        homeButtonTimer.reset();
      }
    } else {
      homeButtonTimer.stop();
      homeButtonTimer.reset();
    }

    if (!coastButton.get() && Constants.currentMode != RobotMode.SIM) {
      RobotContainer.getSuperstructure().CoastMotors();
      DriverStation.reportWarning("Coast Mode Trying To Activate", false);
      coastButtonTimer.start();
      // button is pressed in
    }

    if (coastButtonTimer.hasElapsed(0.1)) {
      RobotContainer.getSuperstructure().CoastMotors();
    }

    if (coastButtonTimer.hasElapsed(10)) {
      DriverStation.reportWarning("Break Mode Trying To Activate", false);
      RobotContainer.getSuperstructure().BreakMotors();
      coastButtonTimer.stop();
      coastButtonTimer.reset();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
