package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
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

  /** meters, distance from hub to drive to for launching. */
  private static final double LAUNCH_DISTANCE = Units.feetToMeters(4.5);

  /** meters, distance from hub for the approach for launching. */
  private static final double APPROACH_DISTANCE = LAUNCH_DISTANCE + Units.feetToMeters(1.0);

  /** Positions for entry and exist of the trench in meters. */
  private static final double BLUE_TRENCH_NEUTRAL_X = 6.2;

  private static final double BLUE_TRENCH_BLUE_X = 3.2;
  private static final double TRENCH_Y = 0.625;

  private StructArrayPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("Target Pose", Pose2d.struct).publish();

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

  /**
   * Create a command to drive to a position at a fixed distance from the hub for launch.
   *
   * @return the command to drive to the launch position
   */
  public Command createDriveLaunchCommand() {

    var rotationToHub = getAngleToHub();
    double angleToHub;
    if (drivebase.isRedAlliance()) {
      angleToHub =
          MathUtil.clamp(rotationToHub.minus(Rotation2d.fromDegrees(180)).getDegrees(), -45, 45)
              + 180.0;
    } else {
      angleToHub = MathUtil.clamp(rotationToHub.getDegrees(), -45, 45);
    }

    return finalDriveLaunchCommand(Math.toRadians(angleToHub));
  }

  /**
   * Create a command to drive to a the final position at a fixed distance and given angle from the
   * hub for launch.
   *
   * @return the command to drive to the launch position
   */
  public Command finalDriveLaunchCommand(double angleToHub) {
    Pose2d hubTarget = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;

    Rotation2d rotationToHub = new Rotation2d(angleToHub);

    var targetPose1 =
        new Pose2d(
            hubTarget
                .getTranslation()
                .minus(new Translation2d(APPROACH_DISTANCE, 0.0).rotateBy(rotationToHub)),
            rotationToHub.minus(new Rotation2d(Math.toRadians(180))));
    var targetPose2 =
        new Pose2d(
            hubTarget
                .getTranslation()
                .minus(new Translation2d(LAUNCH_DISTANCE, 0.0).rotateBy(rotationToHub)),
            rotationToHub.minus(new Rotation2d(Math.toRadians(180))));

    return Commands.runOnce(() -> posePublisher.set(new Pose2d[] {targetPose1, targetPose2}))
        .andThen(drivebase.driveToPosePID(targetPose1, targetPose2))
        .andThen(fuel.launchCommand());
  }

  /**
   * Create a command to drive to through the trench from the neutral zone.
   *
   * @return the command to drive to the launch position
   */
  public Command createDriveTrenchCommand(boolean useLeft) {
    double angleToHub;

    try {

      // Create a list of waypoints from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic
      // rotation.
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              new Pose2d(BLUE_TRENCH_NEUTRAL_X, TRENCH_Y, Rotation2d.fromDegrees(180)),
              new Pose2d(BLUE_TRENCH_BLUE_X, TRENCH_Y, Rotation2d.fromDegrees(180)));

      PathConstraints constraints =
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

      // Create the path using the waypoints created above
      PathPlannerPath path =
          new PathPlannerPath(
              waypoints,
              constraints,
              null, // The ideal starting state, this is not relevant for on-the-fly paths.
              new GoalEndState(1.5, Rotation2d.fromDegrees(90))); // Goal end state.

      // Mirror the path if we want the left side. Alliance flipping is automatic.
      if (useLeft) {
        path = path.mirrorPath();
      }

      // Determine the final angle to the hub based on alliance and trench side
      if (isRedAlliance()) {
        if (useLeft) {
          angleToHub = Math.toRadians(135);
        } else {
          angleToHub = Math.toRadians(-135);
        }
      } else {
        if (useLeft) {
          angleToHub = Math.toRadians(-45);
        } else {
          angleToHub = Math.toRadians(45);
        }
      }

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindThenFollowPath(path, DriveConstants.DRIVE_POSE_CONSTRAINTS)
          .andThen(finalDriveLaunchCommand(angleToHub));

    } catch (Exception e) {
      // Handle exception as needed
      DataLogManager.log("Error loading path: " + e.getMessage());
      return Commands.none(); // Return a no-op command if path loading fails
    }
  }

  /** Periodic function to update SmartDashboard values. */
  public void periodic() {
    SmartDashboard.putNumber("Hub/Distance", getDistanceToHub());
    SmartDashboard.putNumber("Hub/Angle", getAngleToHub().getDegrees());
  }
}
