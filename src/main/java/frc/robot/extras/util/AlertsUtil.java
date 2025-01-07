package frc.robot.extras.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AlertsUtil {
    public static Alert bind(Alert alert, BooleanSupplier event) {
        return bind(alert, event, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public static Alert bind(Alert alert, BooleanSupplier event, EventLoop loop) {
        loop.bind(()->alert.set(event.getAsBoolean()));
        return alert;
    }
}