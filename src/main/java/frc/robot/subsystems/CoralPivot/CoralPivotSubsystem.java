package frc.robot.subsystems.CoralPivot;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivotSubsystem extends SubsystemBase{
    private CoralPivotInterface coralPivotInterface;
    private CoralPivotInterfaceAutoLogged inputs = new CoralPivotInputsAutoLogged();

    public CoralPivotSubsystem(CoralPivotInterface coralPivotInterface){
        this.coralPivotInterface = coralPivotInterface;
    }

    public void setCoralSpeed(double speed){
        coralPivotInterface.setCoralSpeed(speed);
    }

    public void setCoralAngle(double angle){
        coralPivotInterface.setCoralAngle(angle);
    }

    public void setVoltage(double voltage){
        coralPivotInterface.setVoltage(voltage);
    }

    public void periodic(){
        coralPivotInterface.updateInputs(coralPivotInterface);
        Logger.processInputs("CoralPivotSubsystem/", inputs);
    }
}
