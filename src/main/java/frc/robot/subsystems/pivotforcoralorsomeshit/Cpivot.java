// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivotforcoralorsomeshit;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cpivot extends SubsystemBase {
  /** Creates a new Cpivot. */
  public TalonFX pivotMotor = new TalonFX(Constants.CORAL_PIVOT_MOTOR);
  private static final int TICKS_PER_ROTATION = 2048;
  private static final double GEAR_RATIO = 10.0;
  
  public Cpivot(int motorID) {
    pivotMotor = new TalonFX(motorID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = 0.0;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.0;
    config.slot0.kF = 0.0;
    config.motionCruiseVelocity = 15000;
    config.motionAcceleration = 6000;
    pivotMotor.configAllSettings(config);
    pivotMotor.setNeutralMode(NeutralMode.Brake);  
    pivotMotor.setSelectedSensorPosition(0);
    }

    public void setPivotPosition(double degrees) {
        double ticks = degreesToTicks(degrees);
        pivotMotor.set(ControlMode.MotionMagic, ticks);
    }

    public double getCurrentPosition() {
        return ticksToDegrees(pivotMotor.getSelectedSensorPosition());
    }

    public void stop() {
        pivotMotor.set(ControlMode.PercentOutput, 0);
    }

    private double degreesToTicks(double degrees) {
        return degrees / 360.0 * TICKS_PER_ROTATION * GEAR_RATIO;
    }

    private double ticksToDegrees(double ticks) {
        return ticks / (TICKS_PER_ROTATION * GEAR_RATIO) * 360.0;
    }

    @Override
    public void periodic() {
        System.out.println("Current Position: " + getCurrentPosition());
    }
}
