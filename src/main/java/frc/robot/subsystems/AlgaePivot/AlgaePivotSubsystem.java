package frc.robot.subsystems.AlgaePivot;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaePivot.AlgaePivotInterface.AlgaePivotInputs;

public class AlgaePivotSubsystem extends SubsystemBase{
    private AlgaePivotInterface algaePivotInterface;
    private AlgaePivotInputsAutoLogged inputs = AlgaePivotInputsAutoLogged();

    public AlgaePivotSubsystem(AlgaePivotInterface algaePivotInterface){
        this.algaePivotInterface = algaePivotInterface;
    }

    public void setAlgaeSpeed(double speed){
        algaePivotInterface.setAlgaeSpeed(speed);
    }

    public void setAlgaeAngle(double angle){
        algaePivotInterface.setAlgaeAngle(angle);
    }

    public void setVoltage(double voltage){
        algaePivotInterface.setVoltage(voltage);
    }

    public void periodic(){
        algaePivotInterface.updateInputs(inputs);
        Logger.processInputs("AlgaePivotSubsystem/", inputs);
    }
}
