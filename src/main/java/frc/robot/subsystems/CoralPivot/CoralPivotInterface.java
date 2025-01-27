package frc.robot.subsystems.CoralPivot;

import org.littletonrobotics.junction.AutoLog;

public interface CoralPivotInterface {
@AutoLog
    public static class coralPivotInputs{
        public boolean isConnected = true;
        public double coralAngle = 0.0;
        public double coralVoltage = 0.0;
        public double coralVelocity = 0.0;
        public double coralTemp = 0.0;
        public double coralSupplyCurrentAmps = 0.0;
        public double coralTorqueCurrentAmps = 0.0;
    }

    default void updateInputs(coralPivotInputs inputs){}

    default void setCoralSpeed(double speed){}

    default void setCoralAngle(double angle){}

    default void setVoltage(double voltage){}

    default double getCoralAngle(){
        return 0.0;
    }

    default double getCoralPivotTarget(){
        return 0.0;
    }
}
