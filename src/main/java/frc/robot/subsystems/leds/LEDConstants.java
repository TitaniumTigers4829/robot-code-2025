package frc.robot.subsystems.leds;

public final class LEDConstants {
  public static final int LED_PORT = 0;

  public static final class SparkConstants {
    // This subclass contains the constant values for the LED patterns.
    public static final double RAINBOW = -0.99;
    public static final double SHOT_RED = -0.85;
    public static final double SHOT_BLUE = -0.83;
    public static final double SHOT_WHITE = -0.81;
    public static final double RED_ALLIANCE_BLINKIN = -0.39;
    public static final double RAINBOW_WAVE = -0.45;
    public static final double OCEAN = -0.41;
    public static final double BOUNCE_RED = -0.35;
    public static final double BOUNCE_GRAY = -0.33;
    public static final double HEARTBEAT_RED = -0.25;
    public static final double HEARTBEAT_GRAY = -0.19;
    public static final double STROBE_RED = -0.11;
    public static final double STROBE_BLUE = -0.09;
    public static final double STROBE_GOLD = -0.07;
    public static final double STROBE_WHITE = -0.05;

    public static final double HEARTBEAT_1 = 0.43;
    public static final double HEARTBEAT_2 = 0.27;

    public static final double MAGENTA = 0.57;
    public static final double DARK_RED = 0.59;
    public static final double RED = 0.61;
    public static final double VERMILION = 0.63;
    public static final double ORANGE = 0.65;
    public static final double GOLD = 0.67;
    public static final double YELLOW = 0.69;
    public static final double LAWN_GREEN = 0.71;
    public static final double LIME = 0.73;
    public static final double DARK_GREEN = 0.75;
    public static final double GREEN = 0.77;
    public static final double CYAN = 0.79;
    public static final double AQUA = 0.81;
    public static final double SKY_BLUE = 0.83;
    public static final double DARK_BLUE = 0.85;
    public static final double BLUE = 0.87;
    public static final double INDIGO = 0.89;
    public static final double PURPLE = 0.91;
    public static final double WHITE = 0.93;
    public static final double GRAY = 0.95;
    public static final double DARK_GRAY = 0.97;
    public static final double BLACK = 0.99;
  }

  public enum LEDProcess {
    /** alliance color */
    ALLIANCE_COLOR(0, 0, 0, 0),
    /** default */
    DEFAULT(0, 0, 0, 0),
    RAINBOW(SparkConstants.RAINBOW, 0, 0, 0),
    RED_ALLIANCE(SparkConstants.RED_ALLIANCE_BLINKIN, 255, 0, 0),
    BLUE_ALLIANCE(SparkConstants.OCEAN, 0, 0, 255),
    SHOOT(SparkConstants.WHITE, 0, 0, 255),
    OFF(SparkConstants.BLACK, 0, 0, 0),
    AUTONOMOUS(SparkConstants.SHOT_WHITE, 0, 0, 0),
    REVERSE_INTAKE(SparkConstants.RED, 0, 255, 0),
    FINISH_LINE_UP(SparkConstants.GREEN, 255, 255, 255),
    GREEN(SparkConstants.GREEN, 0, 255, 0),
    RED(SparkConstants.RED, 255, 0, 0),
    INTAKE(SparkConstants.RED, 255, 0, 0),
    NOTE_IN(SparkConstants.GREEN, 0, 255, 0),
    NOTE_HALFWAY_IN(SparkConstants.YELLOW, 255, 255, 0);

    public final double sparkValue;
    private final int red, green, blue;

    LEDProcess(double sparkValue, int red, int green, int blue) {
      this.sparkValue = sparkValue;
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}
