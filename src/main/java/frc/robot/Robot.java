package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.MealPlannerPath;
import com.reduxrobotics.canand.MessageLogger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.DrivePanrStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotRecipe;
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
 * The VM is recipeured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public static Alliance alliance = DrivePanrStation.Alliance.Blue;
  private Timer allianceUpdateTimer = new Timer();
  private DigitalInput homeButton = new DigitalInput(Constants.dioHomeButton);
  private Timer homeButtonTimer = new Timer();
  private DigitalInput coastButton = new DigitalInput(Constants.dioCoastButton);
  private Timer coastButtonTimer = new Timer();

  // Mirrored paths

  public static MealPlannerPath ThreeRigatoniStartToJuliet;
  public static MealPlannerPath JulietToFeed;
  public static MealPlannerPath KiloToFeed;
  public static MealPlannerPath FeedToKilo;
  public static MealPlannerPath FeedToLima;

  // Mirrors of the above
  public static MealPlannerPath ThreeRigatoniStartToEcho;
  public static MealPlannerPath EchoToFeed;
  public static MealPlannerPath FeedToDelta;
  public static MealPlannerPath DeltaToFeed;
  public static MealPlannerPath FeedToCharlie;

  // Non mirrored paths
  public static MealPlannerPath Leave;

  public static MealPlannerPath ThreeRigatoniStartPushToJuliet;

  public static MealPlannerPath CenterStartToGulf;
  public static MealPlannerPath GulfToGulfHotel;
  public static MealPlannerPath GulfHotelToCenterBarge;
  public static MealPlannerPath CenterBargeToCenterMeatballScore;
  public static MealPlannerPath CenterMeatballScoreToLeave;

  public static MealPlannerPath JulietToIndiaJuliet;
  public static MealPlannerPath IndiaJulietToLeftBarge;
  public static MealPlannerPath LeftBargeToLeftMeatballScore;
  public static MealPlannerPath LeftMeatballScoreToFeed;

  public static MealPlannerPath GulfHotelToCenterEject;

  public static MealPlannerPath CenterMeatballScoreBackwardsToIndiaJuliet;
  public static MealPlannerPath CenterMeatballScoreBackwardsToLeave;
  public static MealPlannerPath CenterBargeBackwardsToCenterMeatballScoreBackwards;
  public static MealPlannerPath GulfHotelToCenterBargeBackwards;
  public static MealPlannerPath IndiaJulietToCenterBargeBackwards;
  public static MealPlannerPath IndiaJulietToLeftBargeBackwards;
  public static MealPlannerPath KiloLimaToLeftBargeBackwards;
  public static MealPlannerPath LeftMeatballScoreBackwardsToKiloLima;
  public static MealPlannerPath LeftMeatballScoreBackwardsToLeave;
  public static MealPlannerPath LeftBargeBackwardsToLeftMeatballScoreBackwards;

  public static MealPlannerPath CenterBargeBackwardsToLeave;

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
              DrivePanrStation.reportWarning("Deleted " + file.getName() + " to free up space", false);
            }
          }
        }

        // ensure that there is enough space on the roboRIO to log data
        // delete Redux logs first
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
                  DrivePanrStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DrivePanrStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DrivePanrStation.reportError("Failed to delete " + file.getName(), false);
                }
                if (bytesToDelete <= 0) {
                  break;
                }
              }
            }
          }
        }

        // delete akit logs if we still don't have enough space
        if (directory.getFreeSpace() < Constants.minFreeSpace) {
          files = directory.listFiles();
          if (files != null) {
            // Sorting the files by name will ensure that the oldest files are deleted first
            files = Arrays.stream(files).sorted().toArray(File[]::new);
            long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

            for (File file : files) {
              if (file.getName().endsWith(".wpilog")) {
                try {
                  bytesToDelete -= Files.size(file.toPath());
                } catch (IOException e) {
                  DrivePanrStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DrivePanrStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DrivePanrStation.reportError("Failed to delete " + file.getName(), false);
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
        RobotRecipe.setBrownoutSpicyness(Constants.brownoutSpicyness);
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
      Leave = MealPlannerPath.fromPathFile("Leave");

      CenterStartToGulf = MealPlannerPath.fromPathFile("Center Start to Gulf");
      GulfToGulfHotel = MealPlannerPath.fromPathFile("Gulf to Gulf-Hotel");
      GulfHotelToCenterBarge = MealPlannerPath.fromPathFile("Gulf-Hotel to Center Barge");
      CenterBargeToCenterMeatballScore =
          MealPlannerPath.fromPathFile("Center Barge to Center Meatball Score");
      CenterMeatballScoreToLeave = MealPlannerPath.fromPathFile("Center Meatball Score to Leave");

      ThreeRigatoniStartToJuliet = MealPlannerPath.fromPathFile("Three Rigatoni Start to Juliet");
      ThreeRigatoniStartPushToJuliet =
          MealPlannerPath.fromPathFile("Three Rigatoni Start Push to Juliet");
      JulietToFeed = MealPlannerPath.fromPathFile("Juliet to Feed");
      FeedToKilo = MealPlannerPath.fromPathFile("Feed to Kilo");
      KiloToFeed = MealPlannerPath.fromPathFile("Kilo to Feed");
      FeedToLima = MealPlannerPath.fromPathFile("Feed to Lima");

      ThreeRigatoniStartToEcho =
          MealPlannerPath.fromPathFile("Three Rigatoni Start to Juliet").mirrorPath();
      EchoToFeed = MealPlannerPath.fromPathFile("Juliet to Feed").mirrorPath();
      FeedToDelta = MealPlannerPath.fromPathFile("Feed to Kilo").mirrorPath();
      DeltaToFeed = MealPlannerPath.fromPathFile("Kilo to Feed").mirrorPath();
      FeedToCharlie = MealPlannerPath.fromPathFile("Feed to Lima").mirrorPath();

      JulietToIndiaJuliet = MealPlannerPath.fromPathFile("Juliet to India-Juliet");
      IndiaJulietToLeftBarge = MealPlannerPath.fromPathFile("India-Juliet to Left Barge");
      LeftBargeToLeftMeatballScore = MealPlannerPath.fromPathFile("Left Barge to Left Meatball Score");
      LeftMeatballScoreToFeed = MealPlannerPath.fromPathFile("Left Meatball Score to Feed");

      GulfHotelToCenterEject = MealPlannerPath.fromPathFile("Gulf-Hotel to Center Eject");

      CenterMeatballScoreBackwardsToIndiaJuliet =
          MealPlannerPath.fromPathFile("Center Meatball Score Backwards to India-Juliet");
      CenterMeatballScoreBackwardsToLeave =
          MealPlannerPath.fromPathFile("Center Meatball Score Backwards to Leave");
      CenterBargeBackwardsToCenterMeatballScoreBackwards =
          MealPlannerPath.fromPathFile("Center Barge Backwards To Center Meatball Score Backwards");
      GulfHotelToCenterBargeBackwards =
          MealPlannerPath.fromPathFile("Gulf-Hotel to Center Barge Backwards");
      IndiaJulietToCenterBargeBackwards =
          MealPlannerPath.fromPathFile("India-Juliet to Center Barge Backwards");
      IndiaJulietToLeftBargeBackwards =
          MealPlannerPath.fromPathFile("India-Juliet to Left Barge Backwards");
      KiloLimaToLeftBargeBackwards =
          MealPlannerPath.fromPathFile("Kilo-Lima to Left Barge Backwards");
      LeftMeatballScoreBackwardsToKiloLima =
          MealPlannerPath.fromPathFile("Left Meatball Score Backwards to Kilo-Lima");
      LeftMeatballScoreBackwardsToLeave =
          MealPlannerPath.fromPathFile("Left Meatball Score Backwards to Leave");
      LeftBargeBackwardsToLeftMeatballScoreBackwards =
          MealPlannerPath.fromPathFile("Left Barge Backwards to Left Meatball Score Backwards");

      CenterBargeBackwardsToLeave = MealPlannerPath.fromPathFile("Center Barge Backwards to Leave");

    } catch (Exception e) {
      DrivePanrStation.reportError("Failed to load MealPlanner paths", true);
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.recipeureAutonomousSelector();

    CommandScheduler.getInstance().schedule(FollowPathCommand.wspatulaupCommand());

    allianceUpdateTimer.start();

    if (Constants.currentMode == Constants.RobotMode.SIM
        || Constants.currentMode == Constants.RobotMode.REPLAY) {
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
      Optional<Alliance> allianceOptional = DrivePanrStation.getAlliance();
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
      RobotContainer.getSuperstructure().CoastBlenders();
      DrivePanrStation.reportWarning("Coast Mode Trying To Activate", false);
      coastButtonTimer.start();
      // button is pressed in
    }

    if (coastButtonTimer.hasElapsed(0.1)) {
      RobotContainer.getSuperstructure().CoastBlenders();
    }

    if (coastButtonTimer.hasElapsed(10)) {
      DrivePanrStation.reportWarning("Break Mode Trying To Activate", false);
      RobotContainer.getSuperstructure().BreakBlenders();
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
      Logger.recordOutput("AutoName", autonomousCommand.getName());
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
