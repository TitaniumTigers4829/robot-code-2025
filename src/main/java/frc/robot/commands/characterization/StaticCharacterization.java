package frc.robot.commands.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.extras.logging.LoggedTunableNumber;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Characterizes the Subsystem using a current input (setter) and a velocity supplier (getter). This
 * will help tune kS for TorqueCurrentFOC control modes.
 */
public class StaticCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public StaticCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.getAsDouble();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.getAsDouble();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput);
    inputConsumer.accept(0);
  }
}
