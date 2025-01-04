package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

public final class VisionConstants {
  public enum Limelight {
    SHOOTER(0, VisionConstants.SHOOTER_LIMELIGHT_NAME),
    FRONT_LEFT(1, VisionConstants.FRONT_LEFT_LIMELIGHT_NAME),
    FRONT_RIGHT(2, VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME);

    private final int id;
    private final String name;

    Limelight(int id, String name) {
      this.id = id;
      this.name = name;
    }

    public int getId() {
      return id;
    }

    public String getName() {
      return name;
    }

    public static Limelight fromId(int id) {
      return switch (id) {
        case 0 -> SHOOTER;
        case 1 -> FRONT_LEFT;
        case 2 -> FRONT_RIGHT;
        default -> throw new IllegalArgumentException("Invalid Limelight ID: " + id);
      };
    }
  }

  public static final Transform3d SHOOTER_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.3119324724, 0.0, 0.1865472012), new Rotation3d(0.0, 35, 180.0));
  public static final Transform3d FRONT_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.2749477356, -0.269958439, 0.2318054546),
          new Rotation3d(0.0, 25, -35));
  public static final Transform3d FRONT_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.2816630892, 0.2724405524, 0.232156), new Rotation3d(0.0, 25, 35));

  public static final PhotonCamera SHOOTER_CAMERA = new PhotonCamera(Limelight.SHOOTER.getName());
  public static final PhotonCamera FRONT_LEFT_CAMERA =
      new PhotonCamera(Limelight.FRONT_LEFT.getName());
  public static final PhotonCamera FRONT_RIGHT_CAMERA =
      new PhotonCamera(Limelight.FRONT_RIGHT.getName());

  public static final int THREAD_SLEEP_MS = 20;

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double VISION_X_POS_TRUST = 0.5; // meters
  public static final double VISION_Y_POS_TRUST = 0.5; // meters
  public static final double VISION_ANGLE_TRUST = Units.degreesToRadians(50); // radians

  public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 2;
  public static final double LL3_FOV_MARGIN_OF_ERROR = 27;
  public static final double LL3G_FOV_MARGIN_OF_ERROR = 38;

  public static final double MEGA_TAG_2_MAX_HEADING_RATE =
      180; // degrees/s // TODO: This can be tested more

  public static final double MEGA_TAG_2_DISTANCE_THRESHOLD = 5; // TODO: Tune

  public static final double MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD = 0.5; // TODO: tune
  public static final double MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD = 45;

  public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
  public static final int SHOOTER_LIMELIGHT_NUMBER = 0;
  public static final String FRONT_LEFT_LIMELIGHT_NAME = "limelight-left";
  public static final int FRONT_LEFT_LIMELIGHT_NUMBER = 1;
  public static final String FRONT_RIGHT_LIMELIGHT_NAME = "limelight-right";
  public static final int FRONT_RIGHT_LIMELIGHT_NUMBER = 2;

  public static final int MIN_APRIL_TAG_ID = 1;
  public static final int MAX_APRIL_TAG_ID = 16;

  public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 2
    {1.5, 0.02, 0.02, Units.degreesToRadians(180000)}, // 5
    {3, 1.2, 1.2, Units.degreesToRadians(180000)}, // 25
    {4.5, 5.5, 5.5, Units.degreesToRadians(180000)}, // 90
    {8, 10.0, 10.0, Units.degreesToRadians(180000)} // 180
  };

  public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.5
    {1.5, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.7
    {3, 0.03, 0.03, Units.degreesToRadians(180000)}, // 4
    {4.5, 0.06, 0.06, Units.degreesToRadians(180000)}, // 30
    {8, 0.5, 0.5, Units.degreesToRadians(180000)}, // 90
    {10, 10.0, 10.0, Units.degreesToRadians(180000)} // 90
  };

  // Note Detection Lookup Table
  public static final double[][] noteDetectionLookupTable = {
    {154.27134704589844, 171.6876983642578, Units.inchesToMeters(0), Units.inchesToMeters(7.5)},
    {178.7086181640625, 170.32672119140625, Units.inchesToMeters(4), Units.inchesToMeters(7.5)},
    {205.19854736328125, 167.13938903808594, Units.inchesToMeters(8), Units.inchesToMeters(7.5)},
    {228.66574096679688, 161.6444854736328, Units.inchesToMeters(12), Units.inchesToMeters(7.5)},
    {246.21170043945312, 157.2861328125, Units.inchesToMeters(16), Units.inchesToMeters(7.5)},
    {259.782958984375, 150.37551879882812, Units.inchesToMeters(20), Units.inchesToMeters(7.5)},
    {272.013427734375, 146.05697631835938, Units.inchesToMeters(24), Units.inchesToMeters(7.5)},
    {155.65867614746094, 137.33753967285156, Units.inchesToMeters(0), Units.inchesToMeters(13.5)},
    {179.44638061523438, 137.10073852539062, Units.inchesToMeters(4), Units.inchesToMeters(13.5)},
    {202.794921875, 136.00408935546875, Units.inchesToMeters(8), Units.inchesToMeters(13.5)},
    {221.5397491455078, 133.6504364013672, Units.inchesToMeters(12), Units.inchesToMeters(13.5)},
    {238.55738830566406, 131.46266174316406, Units.inchesToMeters(16), Units.inchesToMeters(13.5)},
    {251.65420532226562, 128.71676635742188, Units.inchesToMeters(20), Units.inchesToMeters(13.5)},
    {262.49658203125, 126.91678619384766, Units.inchesToMeters(24), Units.inchesToMeters(13.5)},
    {155.79156494140625, 110.67668914794922, Units.inchesToMeters(0), Units.inchesToMeters(19.5)},
    {176.4880828857422, 111.09066009521484, Units.inchesToMeters(4), Units.inchesToMeters(19.5)},
    {197.07095336914062, 111.10057067871094, Units.inchesToMeters(8), Units.inchesToMeters(19.5)},
    {213.2230682373047, 111.08323669433594, Units.inchesToMeters(12), Units.inchesToMeters(19.5)},
    {227.31912231445312, 110.8406982421875, Units.inchesToMeters(16), Units.inchesToMeters(19.5)},
    {241.66949462890625, 110.21771240234375, Units.inchesToMeters(20), Units.inchesToMeters(19.5)},
    {252.04759216308594, 109.3546142578125, Units.inchesToMeters(24), Units.inchesToMeters(19.5)},
    {156.43348693847656, 92.26092529296875, Units.inchesToMeters(0), Units.inchesToMeters(25.5)},
    {173.71957397460938, 92.69190216064453, Units.inchesToMeters(4), Units.inchesToMeters(25.5)},
    {191.1551513671875, 93.2912826538086, Units.inchesToMeters(8), Units.inchesToMeters(25.5)},
    {205.46400451660156, 93.71550750732422, Units.inchesToMeters(12), Units.inchesToMeters(25.5)},
    {220.07833862304688, 94.37662506103516, Units.inchesToMeters(16), Units.inchesToMeters(25.5)},
    {232.0174560546875, 94.9604263305664, Units.inchesToMeters(20), Units.inchesToMeters(25.5)},
    {242.53091430664062, 95.50878143310547, Units.inchesToMeters(24), Units.inchesToMeters(25.5)},
    {156.4423828125, 82.93617248535156, Units.inchesToMeters(0), Units.inchesToMeters(29.5)},
    {172.02281188964844, 83.42843627929688, Units.inchesToMeters(4), Units.inchesToMeters(29.5)},
    {187.8604736328125, 84.26107788085938, Units.inchesToMeters(8), Units.inchesToMeters(29.5)},
    {201.43931579589844, 85.0326156616211, Units.inchesToMeters(12), Units.inchesToMeters(29.5)},
    {214.3050537109375, 85.8529281616211, Units.inchesToMeters(16), Units.inchesToMeters(29.5)},
    {226.25885009765625, 86.8006820678711, Units.inchesToMeters(20), Units.inchesToMeters(29.5)},
    {236.1659698486328, 87.51233673095703, Units.inchesToMeters(24), Units.inchesToMeters(29.5)},
    {156.423828125, 75.72048950195312, Units.inchesToMeters(0), Units.inchesToMeters(33.5)},
    {170.40736389160156, 76.27440643310547, Units.inchesToMeters(4), Units.inchesToMeters(33.5)},
    {184.97666931152344, 77.00422668457031, Units.inchesToMeters(8), Units.inchesToMeters(33.5)},
    {197.5010223388672, 77.55702209472656, Units.inchesToMeters(12), Units.inchesToMeters(33.5)},
    {209.526611328125, 78.64038848876953, Units.inchesToMeters(16), Units.inchesToMeters(33.5)},
    {220.35401916503906, 79.69290161132812, Units.inchesToMeters(20), Units.inchesToMeters(33.5)},
    {230.5920867919922, 80.84618377685547, Units.inchesToMeters(24), Units.inchesToMeters(33.5)},
    {156.3590087890625, 69.85771179199219, Units.inchesToMeters(0), Units.inchesToMeters(37.5)},
    {169.06935119628906, 70.47810363769531, Units.inchesToMeters(4), Units.inchesToMeters(37.5)},
    {182.52525329589844, 71.03211212158203, Units.inchesToMeters(8), Units.inchesToMeters(37.5)},
    {194.46484375, 72.02528381347656, Units.inchesToMeters(12), Units.inchesToMeters(37.5)},
    {205.57937622070312, 72.911865234375, Units.inchesToMeters(16), Units.inchesToMeters(37.5)},
    {215.69342041015625, 73.98934936523438, Units.inchesToMeters(20), Units.inchesToMeters(37.5)},
    {225.42604064941406, 75.15672302246094, Units.inchesToMeters(24), Units.inchesToMeters(37.5)},
    {156.41311645507812, 64.90918731689453, Units.inchesToMeters(0), Units.inchesToMeters(41.5)},
    {168.29574584960938, 65.42889404296875, Units.inchesToMeters(4), Units.inchesToMeters(41.5)},
    {180.56785583496094, 66.30609893798828, Units.inchesToMeters(8), Units.inchesToMeters(41.5)},
    {191.53038024902344, 66.91743469238281, Units.inchesToMeters(12), Units.inchesToMeters(41.5)},
    {202.05491638183594, 68.10053253173828, Units.inchesToMeters(16), Units.inchesToMeters(41.5)},
    {212.24427795410156, 69.29578399658203, Units.inchesToMeters(20), Units.inchesToMeters(41.5)},
    {221.28753662109375, 70.37397766113281, Units.inchesToMeters(24), Units.inchesToMeters(41.5)},
    {156.75836181640625, 60.81660842895508, Units.inchesToMeters(0), Units.inchesToMeters(45.5)},
    {167.243896484375, 61.163150787353516, Units.inchesToMeters(4), Units.inchesToMeters(45.5)},
    {178.677734375, 61.95714569091797, Units.inchesToMeters(8), Units.inchesToMeters(45.5)},
    {188.78147888183594, 62.63084411621094, Units.inchesToMeters(12), Units.inchesToMeters(45.5)},
    {198.72476196289062, 63.67478561401367, Units.inchesToMeters(16), Units.inchesToMeters(45.5)},
    {207.99429321289062, 64.74365234375, Units.inchesToMeters(20), Units.inchesToMeters(45.5)},
    {216.6978759765625, 65.65827941894531, Units.inchesToMeters(24), Units.inchesToMeters(45.5)},
    {156.3396453857422, 57.33953094482422, Units.inchesToMeters(0), Units.inchesToMeters(49.5)},
    {166.42056274414062, 57.79549789428711, Units.inchesToMeters(4), Units.inchesToMeters(49.5)},
    {177.30941772460938, 58.38725280761719, Units.inchesToMeters(8), Units.inchesToMeters(49.5)},
    {186.7626953125, 59.280269622802734, Units.inchesToMeters(12), Units.inchesToMeters(49.5)},
    {196.08949279785156, 60.245914459228516, Units.inchesToMeters(16), Units.inchesToMeters(49.5)},
    {205.2306671142578, 61.16556930541992, Units.inchesToMeters(20), Units.inchesToMeters(49.5)},
    {213.36785888671875, 62.33674621582031, Units.inchesToMeters(24), Units.inchesToMeters(49.5)},
    {156.1303253173828, 54.62651443481445, Units.inchesToMeters(0), Units.inchesToMeters(53.5)},
    {165.8522491455078, 54.650146484375, Units.inchesToMeters(4), Units.inchesToMeters(53.5)},
    {175.65347290039062, 55.44529724121094, Units.inchesToMeters(8), Units.inchesToMeters(53.5)},
    {184.98977661132812, 56.34360885620117, Units.inchesToMeters(12), Units.inchesToMeters(53.5)},
    {193.63018798828125, 57.18233108520508, Units.inchesToMeters(16), Units.inchesToMeters(53.5)},
    {201.44790649414062, 58.028751373291016, Units.inchesToMeters(20), Units.inchesToMeters(53.5)},
    {209.90614318847656, 59.1672477722168, Units.inchesToMeters(24), Units.inchesToMeters(53.5)},
  };
}
