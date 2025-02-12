package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
  private static final double BASE_X = Units.feetToMeters(3);
  private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
  public static final Mechanism2d MECH_VISUALIZER =
      new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));
  private static final MechanismRoot2d MECH_VISUALIZER_ROOT =
      MECH_VISUALIZER.getRoot("root", BASE_X, Units.inchesToMeters(7.5));
  private static final MechanismRoot2d ARM_PIVOT_BASE =
      MECH_VISUALIZER.getRoot(
          "arm-base", BASE_X - Units.inchesToMeters(10), Units.inchesToMeters(11));
  private static final MechanismRoot2d INTAKE_PIVOT_BASE =
      MECH_VISUALIZER.getRoot(
          "algae-intake-pivot-base",
          BASE_X + Units.inchesToMeters(11.5),
          Units.inchesToMeters(9.5));
  private static final MechanismLigament2d BACK_DRIVETRAIN_HALF =
      new MechanismLigament2d("drive-back", Units.inchesToMeters(14), 180, 4, ORANGE);
  private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF =
      new MechanismLigament2d("drive-front", Units.inchesToMeters(14), 0, 4, ORANGE);

  public static void setupVisualizer() {
    MECH_VISUALIZER_ROOT.append(BACK_DRIVETRAIN_HALF);
    MECH_VISUALIZER_ROOT.append(FRONT_DRIVETRAIN_HALF);
  }

  public static void addAlgaeIntake(MechanismLigament2d intake) {
    INTAKE_PIVOT_BASE.append(intake);
  }

  public static void addAlgaePivot(MechanismLigament2d pivot) {
    ARM_PIVOT_BASE.append(pivot);
  }

  public static void addCoralIntake(MechanismLigament2d intake) {

  }

  public static void setElevatorPosition(double position) {}

}