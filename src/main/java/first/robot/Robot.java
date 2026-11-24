// Copyright (c) 2021-2026 Littleton Robotics
// SPDX-License-Identifier: BSD-3-Clause

package first.robot;

import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Properties;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;

public class Robot extends LoggedRobot {
  private final RobotContainer robotContainer;
  private final RobotDiagnostics diagnostics = new RobotDiagnostics();
  private final Scheduler scheduler = Scheduler.getDefault();

  public Robot() {
    Logger.recordMetadata("ProjectName", "1466 TalonFX Swerve");
    recordBuildMetadata();
    Logger.recordMetadata("RuntimeMode", Constants.currentMode.name());
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.TUNING_MODE));
    switch (Constants.currentMode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }
    Logger.start();
    robotContainer = new RobotContainer();
  }

  private static void recordBuildMetadata() {
    var metadata = new Properties();
    try (var file = Robot.class.getResourceAsStream("/robot-build.properties")) {
      if (file != null) {
        metadata.load(new InputStreamReader(file, StandardCharsets.UTF_8));
      }
    } catch (IOException error) {
      System.err.println("Could not read build metadata: " + error.getMessage());
    }
    for (String key : new String[] {"BuildDate", "GitSHA", "GitDate", "GitBranch", "GitDirty"}) {
      Logger.recordMetadata(key, metadata.getProperty(key, "Unknown"));
    }
  }

  @Override
  public void robotPeriodic() {
    diagnostics.beginLoop();
    robotContainer.periodic();
    diagnostics.recordPhase("InputsAndCoordination");
    scheduler.run();
    diagnostics.recordPhase("Commands");
    robotContainer.periodicAfterScheduler();
    diagnostics.recordPhase("Outputs");
    diagnostics.periodic(scheduler);
  }

  @Override
  public void teleopPeriodic() {
    // RobotContainer handles teleop controls in robotPeriodic alongside the shared robot loop.
  }
}
