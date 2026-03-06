package frc.sim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Dimensions;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FuelSim;

/** Simulate the fuel subsystem DC motors and fuel handling. */
public class FuelSubsystemSim {

  private final CANFuelSubsystem fuelSubsystem;
  private final SwerveSubsystem drivebase;

  private final SparkMaxSim feederSparkSim;
  private final SparkMaxSim launcherSparkSim;
  private final SparkMaxSim intakeSparkSim;
  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;
  private final SparkMax intakeMotor;

  private final DCMotor motorGearbox = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> feederPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, FEEDER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim feederMotorSim = new DCMotorSim(feederPlant, motorGearbox);

  private final LinearSystem<N2, N1, N2> launcherPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, LAUNCHER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim launcherMotorSim = new DCMotorSim(launcherPlant, motorGearbox);

  private final LinearSystem<N2, N1, N2> intakePlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, INTAKE_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim intakeMotorSim = new DCMotorSim(intakePlant, motorGearbox);

  private static final int MAX_BALLS = 12;
  private int ballCount = 0;
  private double launchDelay = 0.0;
  private double ejectDelay = 0.0;
  private double intakeDelay = 0.0;

  /**
   * Create a new FuelSubsystemSim.
   *
   * @param fuelSubsystemToSimulate the CANFuelSubsystem to simulate
   */
  public FuelSubsystemSim(
      CANFuelSubsystem fuelSubsystemToSimulate, SwerveSubsystem drivebaseToSimulate) {

    fuelSubsystem = fuelSubsystemToSimulate;
    drivebase = drivebaseToSimulate;

    launcherMotor = fuelSubsystem.getLauncherMotor();
    feederMotor = fuelSubsystem.getFeederMotor();
    intakeMotor = fuelSubsystem.getIntakeMotor();

    launcherSparkSim = new SparkMaxSim(launcherMotor, motorGearbox);
    feederSparkSim = new SparkMaxSim(feederMotor, motorGearbox);
    intakeSparkSim = new SparkMaxSim(intakeMotor, motorGearbox);

    configureFuelSim();
  }

  /** Update the simulation model. */
  public void updateSim() {
    feederMotorSim.setInput(feederMotor.getAppliedOutput() * feederMotor.getBusVoltage());
    feederMotorSim.update(0.020);
    feederSparkSim.iterate(feederMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

    launcherMotorSim.setInput(launcherMotor.getAppliedOutput() * launcherMotor.getBusVoltage());
    launcherMotorSim.update(0.020);
    launcherSparkSim.iterate(launcherMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

    intakeMotorSim.setInput(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    intakeMotorSim.update(0.020);
    intakeSparkSim.iterate(intakeMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

    // Launch fuel with a delay between launches
    if (launchDelay <= 0.0
        && fuelSubsystem.getLauncherVelocity() > 1000.0
        && fuelSubsystem.getFeederVelocity() > 1000.0) {
      launchFuel();
      launchDelay = TIME_BETWEEN_LAUNCHES;
    } else {
      launchDelay -= 0.02;
    }

    // Eject fuel with a delay between each ejection
    if (ejectDelay <= 0.0
        && fuelSubsystem.getIntakeVelocity() < -1000.0
        && fuelSubsystem.getFeederVelocity() > 1000.0) {
      ejectFuel();
      ejectDelay = TIME_BETWEEN_EJECTS;
    } else {
      ejectDelay -= 0.02;
    }

    // Count down intake delay between
    if (intakeDelay > 0.0) {
      intakeDelay -= 0.02;
    }

    FuelSim.getInstance().updateSim();
    SmartDashboard.putNumber("FuelSim/Ball Count", ballCount);
    SmartDashboard.putNumber("FuelSim/Score", FuelSim.getInstance().getScore());
  }

  /** Method to check if the intake can accept more balls. */
  public boolean canIntakeBalls() {
    return (fuelSubsystem.getIntakeVelocity() > 100)
        && (ballCount < MAX_BALLS)
        && (intakeDelay <= 0.0);
  }

  /** Method to simulate adding a ball to the hopper. */
  public void addBallToHopper() {
    if (ballCount < MAX_BALLS) {
      ballCount++;
      intakeDelay = TIME_BETWEEN_INTAKES;
    }
  }

  // The following methods are adapted from Hammerheads 5000 2026Rebuilt repository
  // for use with FuelSim

  /** A method to simulate launching a single ball from the hopper. */
  public void launchFuel() {
    if (ballCount == 0) {
      return;
    }
    ballCount--;
    Pose3d launchPose = new Pose3d(drivebase.getPose()).transformBy(ROBOT_TO_LAUNCHER_TRANSFORM);
    Translation3d initialPosition = launchPose.getTranslation();
    LinearVelocity linearVel =
        MetersPerSecond.of(
            RPM.of(LAUNCH_RATIO * fuelSubsystem.getLauncherVelocity()).in(RadiansPerSecond)
                * FLYWHEEL_RADIUS.in(Meters));
    Angle angle = Degrees.of(110.0);
    Translation3d initialVelocity = exitVel(linearVel, angle, ROBOT_TO_LAUNCHER_TRANSFORM);
    FuelSim.getInstance().spawnFuel(initialPosition, initialVelocity);

    // Pose3d launchPose = new Pose3d(drivebase.getPose()).plus(ROBOT_TO_LAUNCHER_TRANSFORM);
    // ChassisSpeeds fieldSpeeds = drivebase.getRobotVelocity();
    // LinearVelocity launchVelocity =
    //     MetersPerSecond.of(
    //         RPM.of(LAUNCH_RATIO * fuelSubsystem.getLauncherVelocity()).in(RadiansPerSecond)
    //             * FLYWHEEL_RADIUS.in(Meters));
    // Angle hoodAngle = Degrees.of(110.0);
    // Angle turretYaw = Degrees.of(0.0);
    // double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    // double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    // double xVel = horizontalVel * Math.cos(turretYaw.in(Radians));
    // double yVel = horizontalVel * Math.sin(turretYaw.in(Radians));

    // xVel += fieldSpeeds.vxMetersPerSecond;
    // yVel += fieldSpeeds.vyMetersPerSecond;

    // FuelSim.getInstance()
    //     .spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
  }

  /** A method to simulate ejecting a single ball from the hopper. */
  public void ejectFuel() {
    if (ballCount == 0) {
      return;
    }
    ballCount--;
    Pose3d robot = new Pose3d(drivebase.getPose()).transformBy(ROBOT_TO_EJECT_TRANSFORM);
    Translation3d initialPosition = robot.getTranslation();
    LinearVelocity linearVel = MetersPerSecond.of(1.0);
    Angle angle = Degrees.of(0.0);
    Translation3d initialVelocity = exitVel(linearVel, angle, ROBOT_TO_EJECT_TRANSFORM);
    FuelSim.getInstance().spawnFuel(initialPosition, initialVelocity);
  }

  // A method to calculate the 3D ball velocity of the fuel based on the
  // robot's pose, robot speeds, eject/launch speed and elevation angle
  private Translation3d exitVel(LinearVelocity vel, Angle angle, Transform3d transform) {
    Pose3d robot = new Pose3d(drivebase.getPose()).transformBy(transform);
    ChassisSpeeds fieldSpeeds = drivebase.getRobotVelocity();

    double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
    double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
    double xVel = horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
    double yVel = horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    return new Translation3d(xVel, yVel, verticalVel);
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        Dimensions.FULL_WIDTH.in(Meters),
        Dimensions.FULL_LENGTH.in(Meters),
        Dimensions.BUMPER_HEIGHT.in(Meters),
        drivebase::getPose,
        drivebase::getRobotVelocity);
    instance.registerIntake(
        Dimensions.FULL_LENGTH.div(2).in(Meters),
        Dimensions.FULL_LENGTH.div(2).plus(Inches.of(7)).in(Meters),
        -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
        Dimensions.FULL_WIDTH.div(2).minus(Inches.of(7)).in(Meters),
        this::canIntakeBalls,
        this::addBallToHopper);

    instance.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  // Method to set the ball count for testing purposes
  public void setBallCount(int count) {
    ballCount = Math.max(0, Math.min(count, MAX_BALLS));
  }
}
