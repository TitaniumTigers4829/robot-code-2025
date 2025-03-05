package frc.robot.extras.logging;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class RuntimeLog {
  private static final StringPublisher entry;

  static {
    // we need to make sure we never log network tables through the implicit wpilib logger
    entry = NetworkTableInstance.getDefault().getStringTopic("/Sim/Runtime").publish();
    debug("Sim Runtime Logger Initialized");
    debug("Sim Diagnostics Logger Initialized");
  }

  public static void debug(String debug) {
    entry.set("[SIM] (DEBUG) " + debug);
  }

  public static void info(String info) {
    entry.set("[SIM] (INFO) " + info);
    System.out.println("[SIM] " + info);
  }

  public static void warn(String warning) {
    entry.set("[SIM] (WARNING) " + warning);
    DriverStationJNI.sendError(false, 1, false, "[SIM] " + warning, "", "", true);
  }

  public static void error(String error) {
    entry.set("[SIM] (ERROR) " + error);
    DriverStationJNI.sendError(true, 1, false, "[SIM] " + error, "", "", true);
  }
}
