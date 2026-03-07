// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.LUT;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  /** Set to true to log Joystick data. Set to false otherwise. */
  public static final boolean LOG_JOYSTICK_DATA = true;

  /** Set to true to send telemetry data to Live Window. Set to false to disable it. */
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set a Global Constant to either Show or Hide extended logging data for each of the subsystems
  // Set to true to show extended logging data.
  // Set to false to hide extended logging data.

  /** Set to true to show extended logging data for the drive subsystem. */
  public static final boolean SD_SHOW_DRIVE_EXTENDED_LOGGING_DATA = true;

  /** Set to true to enable loop timing logging. */
  public static final boolean LOOP_TIMING_LOG = false;

  /** Set to true to enable using Tunable Numbers. */
  public static final boolean TUNING_MODE = true;

  /** Set to true to log each frame of command execution. Set to false to disable. */
  public static final boolean COMMAND_EXECUTE_LOG = false;

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    /** USB port ID for the driver controller (primary driver). */
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /** USB port ID for the operator controller (secondary driver). */
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Joystick Deadband
    /** General deadband value for joystick inputs to prevent noise. */
    public static final double DEADBAND = 0.1;

    /** Deadband for the left joystick Y-axis input. */
    public static final double LEFT_Y_DEADBAND = 0.1;

    /** Deadband for the right joystick X-axis input. */
    public static final double RIGHT_X_DEADBAND = 0.1;

    /** Scaling constant applied to turning inputs for smoother control. */
    public static final double TURN_CONSTANT = 6;
  }

  /** Constants used to define the robot's physical dimensions for fuel sim. */
  public static class Dimensions {
    private Dimensions() {
      throw new IllegalStateException("Dimensions Utility Class");
    }

    public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
    public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
    public static final Distance FRAME_SIZE_Y = Inches.of(26.25); // left to right (y-axis)
    public static final Distance FRAME_SIZE_X = Inches.of(28.75); // front to back (x-axis)

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
  }

  /** Constants used for the swerve drive subsystem. */
  public static final class DriveConstants {

    private DriveConstants() {
      throw new IllegalStateException("DriveConstants Utility Class");
    }

    /** Robot mass in kilograms (40lbs converted to kg). */
    public static final double ROBOT_MASS = 40.0 * 0.453592; // 40lbs * kg per pound

    /** Robot chassis matter definition for swervelib physics. */
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

    /** Control loop time in seconds (20ms + 110ms SparkMax velocity lag). */
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag

    /**
     * Maximum robot speed calculated for Neo Vortex motors at 6700 RPM with 6.75:1 gears and 4"
     * wheels.
     */
    public static final double MAX_SPEED = Units.feetToMeters(6700 / 5.9 / 60 * 4 * Math.PI / 12);

    /** Time to hold motor brakes when robot is disabled in seconds. */
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    /** Speed scaling factor for POV/directional drive commands. */
    public static final double POV_SPEED = 0.1;

    /** Enable/disable vision system for odometry correction. */
    public static final boolean ENABLE_VISION = true;

    /** Maximum distance to detect AprilTags in meters. */
    public static final double MAX_TAG_DISTANCE = 2.75; // meters

    /** Maximum acceptable pose ambiguity from AprilTag detection. */
    public static final double MAX_POSE_AMBIGUITY = 0.1;

    /** Whether to use alliance color for field mirror operations. */
    public static final boolean USE_ALLIANCE = true;

    /** Starting pose for blue alliance in meters and degrees. */
    public static final Pose2d BLUE_START_POSE =
        new Pose2d(new Translation2d(Meter.of(3.6), Meter.of(4.0)), Rotation2d.fromDegrees(180));

    /** Starting pose for red alliance in meters and degrees. */
    public static final Pose2d RED_START_POSE =
        new Pose2d(new Translation2d(Meter.of(13.0), Meter.of(4.0)), Rotation2d.fromDegrees(0));

    // Constants for drive to pose initial path following
    /**
     * Path constraints for initial pathfinding (max velocity, acceleration, angular velocity,
     * angular acceleration).
     */
    public static final PathConstraints DRIVE_POSE_CONSTRAINTS =
        new PathConstraints(1.0, 4.0, Units.degreesToRadians(180), Units.degreesToRadians(720));

    /** Distance from target at which to switch to PID control in meters. */
    public static final double DISTANCE_UNTIL_PID = Units.inchesToMeters(3);

    /** Rotation tolerance before switching to PID control in degrees. */
    public static final double ROTATION_GOAL_BEFORE_PID = 1;

    /** Final velocity at end of pathfinding phase in m/s. */
    public static final LinearVelocity PATH_FIND_END_VELOCITY = MetersPerSecond.of(1.0);

    // Constants for drive to pose final Holonomic controller
    /** Rotation tolerance for final positioning in degrees. */
    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(2.0);

    /** Position tolerance for final positioning in centimeters. */
    public static final Distance POSITION_TOLERANCE = Centimeter.of(1.0);

    /** Speed tolerance for final positioning in inches per second. */
    public static final LinearVelocity SPEED_TOLERANCE = InchesPerSecond.of(1);

    /** Debounce time for end trigger detection in seconds. */
    public static final Time END_TRIGGER_DEBOUNCE = Seconds.of(0.1);

    /** Timeout for auto-align adjustments in seconds. */
    public static final Time AUTO_ALIGN_ADJUST_TIMEOUT = Seconds.of(1.0);

    /** PID constants for translational control (P, I, D). */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.5, 0.0, 0.0);

    /** PID constants for rotational control (P, I, D). */
    public static final PIDConstants ROTATION_PID = new PIDConstants(2.5, 0.0, 0.0);
  }

  /** Constants for the fuel subsystem and simulation. */
  public static final class FuelConstants {
    private FuelConstants() {
      // Prevent instantiation
    }

    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 16;
    public static final int LAUNCHER_MOTOR_ID = 15;
    public static final int LAUNCHER_MOTOR2_ID = 17;
    public static final int INTAKE_MOTOR_ID = 14;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 20;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 4.5;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_INTAKE_VOLTAGE = 4;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1.0;
    public static final double EJECTING_FEEDER_VOLTAGE = 12;
    public static final double EJECTING_INTAKE_VOLTAGE = -6;
    public static final double RATE_LIMIT = 999.0; // volts per second
    public static final double ENABLE_LAUNCH_TABLE = 1;
    public static final LUT LAUNCH_TABLE =
        new LUT(new double[][] {{1.5, 2800.0}, {2.0, 3000.0}, {2.5, 3400.0}, {2.95, 3600}});

    // Constants tunable through TunableNumbers
    public static final double LAUNCHER_SPEED_RPM = 3200.0;
    public static final double LAUNCHER_KP_VOLTS_PER_RPM = 0.00015;
    public static final double LAUNCHER_KD_VOLTS_PER_RPM_SEC = 0.0;
    public static final double LAUNCHER_KV_VOLTS_PER_RPM = 0.0021;

    /** Motor simulation constants. */
    public static final double POUND_IN2_TO_KG_METERS2 =
        Units.lbsToKilograms(1) * Math.pow(Units.inchesToMeters(1), 2);

    public static final double FEEDER_MOTOR_MOI_IN_LBS2 = 0.5;
    public static final double FEEDER_MOTOR_MOI_KG_METERS2 =
        FEEDER_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
    public static final double LAUNCHER_MOTOR_MOI_IN_LBS2 = 5.0;
    public static final double LAUNCHER_MOTOR_MOI_KG_METERS2 =
        LAUNCHER_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
    public static final double INTAKE_MOTOR_MOI_IN_LBS2 = 1.0;
    public static final double INTAKE_MOTOR_MOI_KG_METERS2 =
        INTAKE_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;

    public static final Transform3d ROBOT_TO_LAUNCHER_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18.0)), Rotation3d.kZero);
    public static final Transform3d ROBOT_TO_EJECT_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.of(15.0), Inches.zero(), Inches.of(2.0)), Rotation3d.kZero);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
    public static final double TIME_BETWEEN_LAUNCHES = 0.3;
    public static final double TIME_BETWEEN_INTAKES = 0.3;
    public static final double TIME_BETWEEN_EJECTS = 0.3;
    public static final double LAUNCH_RATIO =
        0.375; // Ratio of ball exit velocity launcher to wheel edge speed
  }
}
