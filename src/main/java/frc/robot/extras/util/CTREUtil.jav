package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class CTREUtil {
    public static <U extends Unit, M extends Measure<U>> DoubleSupplier doubSup(Supplier<M> orig, U desiredUnits) {
        return ()->orig.get().in(desiredUnits);
    }
    public static <U extends Unit, M extends Measure<U>> DoubleSupplier doubSup(Supplier<M> orig) {
        return ()->orig.get().magnitude();
    }

    /**
     * DOES NOT REFRESH
     * @param <U>
     * @param <M>
     * @param orig
     * @param desiredUnits
     * @return
     */
    public static <U extends Unit, M extends Measure<U>> DoubleSupplier doubSup(StatusSignal<M> orig, U desiredUnits) {
        Supplier<M> sup = orig::getValue;
        return ()->sup.get().in(desiredUnits);
    }
    public static <U extends Unit, M extends Measure<U>> DoubleSupplier doubSup(StatusSignal<M> orig) {
        Supplier<M> sup = orig::getValue;
        return ()->sup.get().magnitude();
    }
}