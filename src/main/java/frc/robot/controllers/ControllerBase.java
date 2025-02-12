package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class ControllerBase {
  static {
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private final CommandXboxController controller;

  /** Button: 1 */
  protected final Trigger A;

  /** Button: 2 */
  protected final Trigger B;

  /** Button: 3 */
  protected final Trigger X;

  /** Button: 4 */
  protected final Trigger Y;

  /** Left Center; Button: 7 */
  protected final Trigger Back;

  /** Right Center; Button: 8 */
  protected final Trigger Start;

  /** Left Bumper; Button: 5 */
  protected final Trigger LB;

  /** Right Bumper; Button: 6 */
  protected final Trigger RB;

  /** Left Stick; Button: 9 */
  protected final Trigger LS;

  /** Right Stick; Button: 10 */
  protected final Trigger RS;

  /** Left Trigger; Axis: 2 */
  protected final Trigger LT;

  /** Right Trigger; Axis: 3 */
  protected final Trigger RT;

  /** DPad Up; Degrees: 0 */
  protected final Trigger DPU;

  /** DPad Right; Degrees: 90 */
  protected final Trigger DPR;

  /** DPad Down; Degrees: 180 */
  protected final Trigger DPD;

  /** DPad Left; Degrees: 270 */
  protected final Trigger DPL;

  /** for button idx (nice for sim) {@link edu.wpi.first.wpilibj.XboxController.Button} */
  protected ControllerBase(int port, boolean makeController) {
    if (makeController) {
      controller = new CommandXboxController(port);
      A = controller.a();
      B = controller.b();
      X = controller.x();
      Y = controller.y();
      LB = controller.leftBumper();
      RB = controller.rightBumper();
      Back = controller.back();
      Start = controller.start();
      LS = controller.leftStick();
      RS = controller.rightStick();
      LT = controller.leftTrigger(0.25);
      RT = controller.rightTrigger(0.25);
      DPR = controller.povRight();
      DPD = controller.povDown();
      DPL = controller.povLeft();
      DPU = controller.povUp();
    } else {
      controller = null;
      final Trigger t = new Trigger(() -> false);
      A = t;
      B = t;
      X = t;
      Y = t;
      LB = t;
      RB = t;
      Back = t;
      Start = t;
      LS = t;
      RS = t;
      LT = t;
      RT = t;
      DPR = t;
      DPD = t;
      DPL = t;
      DPU = t;
      return;
    }
  }

  private DoubleSupplier deadbandSupplier(DoubleSupplier supplier, double deadband) {
    return () -> MathUtil.applyDeadband(supplier.getAsDouble(), deadband);
  }

  /**
   * Right on the stick is positive (axis 4)
   *
   * @return A supplier for the value of the right stick x axis
   */
  public DoubleSupplier rightStickX() {
    return () -> -controller.getRightX();
  }

  /**
   * Right on the stick is positive (axis 4)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the right stick x axis
   */
  public DoubleSupplier rightStickX(double deadband) {
    return deadbandSupplier(rightStickX(), deadband);
  }

  /**
   * Up on the stick is positive (axis 5)
   *
   * @return A supplier for the value of the right stick y axis
   */
  public DoubleSupplier rightStickY() {
    return controller::getRightY;
  }

  /**
   * Up on the stick is positive (axis 5)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the right stick y axis
   */
  public DoubleSupplier rightStickY(double deadband) {
    return deadbandSupplier(rightStickY(), deadband);
  }

  /**
   * Right on the stick is positive (axis 0)
   *
   * @return A supplier for the value of the left stick x axis
   */
  public DoubleSupplier leftStickX() {
    return () -> -controller.getLeftX();
  }

  /**
   * Right on the stick is positive (axis 0)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the left stick x axis
   */
  public DoubleSupplier leftStickX(double deadband) {
    return deadbandSupplier(leftStickX(), deadband);
  }

  /**
   * Up on the stick is positive (axis 1)
   *
   * @return A supplier for the value of the left stick y axis
   */
  public DoubleSupplier leftStickY() {
    return controller::getLeftY;
  }

  /**
   * Up on the stick is positive (axis 1)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the left stick y axis
   */
  public DoubleSupplier leftStickY(double deadband) {
    return deadbandSupplier(leftStickY(), deadband);
  }

  /**
   * will print warning if this trigger is also bound to a command
   *
   * @param suppressWarning if true will not print warning even if bound to a command
   */
  public DoubleSupplier rightTrigger(boolean suppressWarning) {
    return controller::getRightTriggerAxis;
  }

  /**
   * will print warning if this trigger is also bound to a command
   *
   * @param suppressWarning if true will not print warning even if bound to a command
   */
  public DoubleSupplier leftTrigger(boolean suppressWarning) {
    return controller::getLeftTriggerAxis;
  }

  /**
   * Will rumble both sides of the controller with a magnitude
   *
   * @param magnitude The magnitude to rumble at
   */
  public void rumble(double magnitude) {
    controller.getHID().setRumble(RumbleType.kBothRumble, magnitude);
  }
}
