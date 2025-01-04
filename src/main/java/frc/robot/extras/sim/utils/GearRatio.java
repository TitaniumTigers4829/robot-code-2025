package frc.robot.extras.sim.utils;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * A type safe way of representing gear ratios
 * keeping the context of the gear ratio in mind.
 */
public sealed interface GearRatio extends StructSerializable {
    /**
     * Get the reduction of the gear ratio.
     * 
     * @return the reduction of the gear ratio
     */
    double getReduction();

    /**
     * Get the overdrive of the gear ratio.
     * 
     * @return the overdrive of the gear ratio
     */
    default double getOverdrive() {
        return 1.0 / getReduction();
    }

    /**
     * Combine this gear ratio with another gear ratio.
     * 
     * @param next the next gear ratio to combine with
     * @return the combined gear ratio
     */
    default GearRatio then(GearRatio next) {
        return new Reduction(getReduction() * next.getReduction());
    }

    /**
     * Combine this gear ratio with a reduction.
     * 
     * @param reduction the reduction to combine with
     * @return the combined gear ratio
     */
    default GearRatio thenReduction(double reduction) {
        return new Reduction(getReduction() * reduction);
    }

    /**
     * Combine this gear ratio with an overdrive.
     * 
     * @param overdrive the overdrive to combine with
     * @return the combined gear ratio
     */
    default GearRatio thenOverdrive(double overdrive) {
        return new Overdrive(getOverdrive() * overdrive);
    }

    /**
     * A type safe representation of a gearing reduction.
     */
    public record Reduction(double reduction) implements GearRatio {
        @Override
        public double getReduction() {
            return reduction;
        }

        public static final GearRatioStruct struct = new GearRatioStruct();
    }

    /**
     * A type safe representation of a gearing overdrive.
     */
    public record Overdrive(double overdrive) implements GearRatio {
        @Override
        public double getReduction() {
            return 1.0 / overdrive;
        }

        public static final GearRatioStruct struct = new GearRatioStruct();
    }

    /**
     * Create a new gear ratio with the given reduction.
     * 
     * <p> Reduction means a number above 1.0 causes the output to be slower than the input.
     * 
     * @param reduction the reduction of the gear ratio
     * @return a new instance of {@link Reduction}
     */
    public static Reduction reduction(double reduction) {
        return new Reduction(reduction);
    }

    /**
     * Create a new gear ratio with the given overdrive.
     * 
     * <p> Overdrive means a number above 1.0 causes the output to be faster than the input.
     * 
     * @param overdrive the overdrive of the gear ratio
     * @return a new instance of {@link Overdrive}
     */
    public static Overdrive overdrive(double overdrive) {
        return new Overdrive(overdrive);
    }

    public static final GearRatioStruct struct = new GearRatioStruct();

    public static final class GearRatioStruct implements Struct<GearRatio> {
        @Override
        public String getSchema() {
            return "float64 reduction";
        }

        @Override
        public int getSize() {
            return Double.BYTES;
        }

        @Override
        public Class<GearRatio> getTypeClass() {
            return GearRatio.class;
        }

        @Override
        public String getTypeName() {
            return "GearRatio";
        }

        @Override
        public void pack(ByteBuffer bb, GearRatio value) {
            bb.putDouble(value.getReduction());
        }

        @Override
        public GearRatio unpack(ByteBuffer bb) {
            return new Reduction(bb.getDouble());
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    } 
}
