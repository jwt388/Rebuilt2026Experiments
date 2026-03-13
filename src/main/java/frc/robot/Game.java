package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/** The Game class contains functions specific to the game. */
public class Game {
  private RobotContainer robotContainer;
  private SwerveSubsystem drivebase;
  private CANFuelSubsystem fuel;

  // **Pose for red alliance to the hub in meters and degrees. */
  public static final Pose2d RED_HUB_CENTER =
      new Pose2d(new Translation2d(11.915, 4.035), new Rotation2d());

  /** Pose for blue alliance to the hub in meters and degrees. */
  public static final Pose2d BLUE_HUB_CENTER =
      new Pose2d(new Translation2d(4.626, 4.035), new Rotation2d());

  private static final double HUB_HEADING_TOL_DEG = 2.5;
  private static final double HUB_MIN_RADIUS_M = Units.feetToMeters(4.0);
  private static final double HUB_MAX_RADIUS_M = Units.feetToMeters(10.0);

  // Offsets for robot when launching and approaching
  private static final double LAUNCH_OFFSET = Units.feetToMeters(4.5);
  private static final double APPROACH_OFFSET = Units.feetToMeters(5.5);

  /** Constructor for the Game class. */
  public Game(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  /** Initialize the game class by setting up subsystems. */
  public void init() {
    this.drivebase = robotContainer.getDriveSubsystem();
    this.fuel = robotContainer.getBallSubsystem();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Calculate the distance to the alliance hub.
   *
   * @return The distance to the alliance hub in meters.
   */
  public double getDistanceToHub() {
    Pose2d hubPos = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return drivebase.getPose().getTranslation().getDistance(hubPos.getTranslation());
  }

  /**
   * Calculate the angle to the alliance hub.
   *
   * @return The angle to the alliance hub as Rotation2d.
   */
  public Rotation2d getAngleToHub() {
    Pose2d hubPos = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return hubPos.getTranslation().minus(drivebase.getPose().getTranslation()).getAngle();
  }

  private Pose2d getHubCenterPose() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return RED_HUB_CENTER;
    }
    return BLUE_HUB_CENTER;
  }

  /**
   * Returns true if the robot is within the defined distance from the hub and aimed towards the
   * hub.
   */
  public boolean isRobotReadyAtHub() {
    Pose2d robotPose = drivebase.getPose();
    Pose2d hubPose = getHubCenterPose();

    if (!isRobotOnAllianceSideOfHub(robotPose, hubPose)) {
      return false;
    }

    Translation2d hubToRobot = robotPose.getTranslation().minus(hubPose.getTranslation());
    double dist = hubToRobot.getNorm();
    if (dist < HUB_MIN_RADIUS_M || dist > HUB_MAX_RADIUS_M) {
      return false;
    }
    Rotation2d desiredHeading = hubToRobot.getAngle(); // direction robot should face
    Rotation2d currentHeading = robotPose.getRotation(); // robot’s current yaw

    double headingErrorDeg = Math.abs(desiredHeading.minus(currentHeading).getDegrees());
    return headingErrorDeg <= HUB_HEADING_TOL_DEG;
  }

  private boolean isRobotOnAllianceSideOfHub(Pose2d robotPose, Pose2d hubPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    double robotX = robotPose.getX();
    double hubX = hubPose.getX();

    if (alliance.get() == Alliance.Blue) {
      return robotX < hubX;
    } else {
      return robotX > hubX;
    }
  }

  public Translation2d getHubToRobotVector() {
    Pose2d robotPose = drivebase.getPose();
    return robotPose.getTranslation().minus(getHubCenterPose().getTranslation());
  }

  public Rotation2d getHubToRobotAngle() {
    return getHubToRobotVector().getAngle();
  }

  /** Command to drive with the launcher aimed at the alliance hub. */
  public Command aimHubDriveCommand(Supplier<ChassisSpeeds> velocity) {
    Pose2d hubTarget = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return drivebase.aimAtPoseCommand(velocity, hubTarget);
  }

  /** Periodic function to update SmartDashboard values. */
  public void periodic() {
    SmartDashboard.putNumber("Hub/Distance", getDistanceToHub());
    SmartDashboard.putNumber("Hub/Angle", getAngleToHub().getDegrees());
  }

  /**
   * Creates a command to drive diagonally in front of the nearest hub. The target is exactly a
   * 7-foot hypotenuse away.
   */
  public Command driveHubCommand() {
    // Ensures everything runs at run time, instead of after
    return Commands.defer(
        () -> {
          // Pose of robot at hub
          Pose2d hubCenter = getHubCenterPose();
          Rotation2d hubAngle = getHubToRobotAngle();

          // Limit the target angle to a range in the alliance zone
          double angleToHub;
          if (drivebase.isRedAlliance()) {
            angleToHub = MathUtil.clamp(hubAngle.getDegrees(), -45, 45);
          } else {
            angleToHub =
                MathUtil.clamp(hubAngle.minus(Rotation2d.fromDegrees(180)).getDegrees(), -45, 45)
                    + 180.0;
          }
          hubAngle = Rotation2d.fromDegrees(angleToHub);

          // Movement for robot to shooting and approach location
          Translation2d launchTranslation = new Translation2d(LAUNCH_OFFSET, hubAngle);
          Translation2d approachTranslation = new Translation2d(APPROACH_OFFSET, hubAngle);

          // Actual robot positions
          Pose2d launchPose =
              new Pose2d(hubCenter.getTranslation().plus(launchTranslation), hubAngle);
          Pose2d approachPose =
              new Pose2d(hubCenter.getTranslation().plus(approachTranslation), hubAngle);

          // Return command to drive
          return drivebase.driveToPosePID(approachPose, launchPose);
        },
        Set.of(drivebase));
  }
}
