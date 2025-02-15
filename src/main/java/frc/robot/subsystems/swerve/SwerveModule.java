package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
import frc.robot.extras.util.LoggedTunableNumber;
import frc.robot.extras.util.Tracer;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

  private final ModuleInterface io;
  private final String name;
  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private final Alert hardwareFaultAlert;
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  // private static final LoggedTunableNumber drivekT =
  //     new LoggedTunableNumber("Drive/Module/DrivekT");
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT, SWERVE_ROBOT -> {
        drivekS.initDefault(5.0);
        drivekV.initDefault(0);
        drivekP.initDefault(35.0);
        drivekD.initDefault(0);
        turnkP.initDefault(4000.0);
        turnkD.initDefault(50.0);
      }
      default -> {
        drivekS.initDefault(0.014);
        drivekV.initDefault(0.134);
        // drivekT.initDefault(0);
        drivekP.initDefault(0.1);
        drivekD.initDefault(0);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0);
      }
    }
  }

  public SwerveModule(ModuleInterface io, String name) {
    this.io = io;
    this.name = name;
    this.hardwareFaultAlert =
        new Alert("Module-" + name + " Hardware Fault", Alert.AlertType.kError);
    this.hardwareFaultAlert.set(false);
  }

  /** Updates the module's odometry inputs. */
  public void updateOdometryInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module-" + name, inputs);
    Tracer.traceFunc("Module-" + name, () -> io.updateInputs(inputs));
    this.hardwareFaultAlert.set(!inputs.isConnected);
  }

  /**
   * Sets the drive voltage of the module.
   *
   * @param volts the voltage to set the drive motor to
   */
  public void setVoltage(Voltage volts) {
    io.setDriveVoltage(volts);
    io.setTurnVoltage(Volts.zero());
  }

  /**
   * Gets the drive voltage of the module.
   *
   * @return the drive voltage of the module
   */
  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  /**
   * Gets the drive velocity of the module.
   *
   * @return the drive velocity of the module
   */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocity;
  }

  /**
   * Sets the desired state of the module. It optimizes this meaning that it will adjust the turn
   * angle to be the shortest path to the desired angle. So rather than turning 170 degrees CW it
   * will turn 10 degrees CCW and invert the motor.
   *
   * @param state
   */
  public void setOptimizedDesiredState(SwerveModuleState state) {
    state.optimize(getTurnRotation());
    if (state.speedMetersPerSecond < 0.01) {
      io.stopModule();
    }
    io.setDesiredState(state);
    Logger.recordOutput("Drive/desired turn angle", state.angle.getRotations());
  }

  /** Stops the module */
  public void stopModule() {
    io.stopModule();
  }

  /**
   * Gets the turn angle of the module.
   *
   * @return the turn angle of the module 0 being forward, CCW being positive
   */
  public Rotation2d getTurnRotation() {
    return inputs.turnAbsolutePosition;
  }

  /**
   * Gets the turn velocity of the module.
   *
   * @return the turn velocity in rotations per second
   */
  public double getTurnVelocity() {
    return inputs.turnVelocity;
  }

  /** Returns the current drive position of the module in meters. */
  public double getDrivePositionMeters() {
    return ModuleConstants.DRIVE_TO_METERS * inputs.drivePosition;
  }

  /**
   * Gets the drive velocity of the module in meters per second.
   *
   * @return the drive velocity in meters per second
   */
  public double getDriveVelocityMetersPerSec() {
    return ModuleConstants.DRIVE_TO_METERS_PER_SECOND * inputs.driveVelocity;
  }

  /**
   * Gets the drive position in radians.
   *
   * @return a double containing current position of the driving motors in radians.
   */
  public double getDrivePositionRadians() {
    return io.getDrivePositionRadians();
  }

  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnRotation());
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is
   * rotated.
   *
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getTurnRotation());
  }

  /**
   * This is called in the periodic method of the SwerveDrive. It is used to update module values
   * periodically
   */
  public void periodic() {
    
    // Update tunable numbers
    if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
      io.setDriveFF(drivekS.get(), drivekV.get(), 0.0);
    }
    if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
      io.setDrivePID(drivekP.get(), 0, drivekD.get());
    }
    if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
      io.setTurnPID(turnkP.get(), 0, turnkD.get());
    }
  }
}
