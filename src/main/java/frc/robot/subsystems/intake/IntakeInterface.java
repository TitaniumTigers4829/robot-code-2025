package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface {

    @AutoLog
    public static class IntakeInputs {
        public boolean isConnected = true;
        public double intakeSpeed = 0.0;
        public double intakeTemp = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(IntakeInputs inputs){}

    default void setSpeed(double speed){}

    default double getSpeed(){
        return 0.0;
    }
}
