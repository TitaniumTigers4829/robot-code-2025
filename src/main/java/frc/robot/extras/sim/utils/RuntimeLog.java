package frc.robot.extras.sim.utils;

import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.NTDataLogger;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class RuntimeLog {
    private static final StringPublisher entry;
    private static final DataLogger diagnostics;

    static {
        // we need to make sure we never log network tables through the implicit wpilib logger
        entry = NetworkTableInstance.getDefault().getStringTopic("/Sham/Runtime").publish();
        debug("Sham Runtime Logger Initialized");
        diagnostics = new NTDataLogger(NetworkTableInstance.getDefault()).getSubLogger("/Sham");
        debug("Sham Diagnostics Logger Initialized");
    }

    public static void debug(String debug) {
        entry.set("[SHAM] (DEBUG) " + debug);
    }

    public static void info(String info) {
        entry.set("[SHAM] (INFO) " + info);
        System.out.println("[SHAM] " + info);
    }

    public static void warn(String warning) {
        entry.set("[SHAM] (WARNING) " + warning);
        DriverStationJNI.sendError(false, 1, false, "[SHAM] " + warning, "", "", true);
    }

    public static void error(String error) {
        entry.set("[SHAM] (ERROR) " + error);
        DriverStationJNI.sendError(true, 1, false, "[SHAM] " + error, "", "", true);
    }

    public static DataLogger loggerFor(String subPath) {
        return diagnostics.getSubLogger(subPath);
    }
}
