// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();
  // The driver's controller
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // The operator's controller
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  // Now all the subsystems.
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final LEDSubsystem led = new LEDSubsystem();

  // Helper functions for this year's game.
  private final Game game = new Game(this);

  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem(game);

  // Define objects for other subsystems here

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverController.getLeftY() * -1,
              () -> driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverController.getRightX() * -1)
          .deadband(OIConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(DriveConstants.USE_ALLIANCE);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity =
      drivebase.driveFieldOriented(driveAngularVelocity).withName("Angular Velocity");

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  // This doesn't do what we want in 2025.1.1
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveRobotOrientedAngularVelocity =
      drivebase.driveFieldOriented(driveRobotOriented).withName("Robot Oriented");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftForwardRobotOriented =
      SwerveInputStream.of(drivebase.getSwerveDrive(), () -> DriveConstants.POV_SPEED, () -> 0.0)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftForward =
      drivebase.driveFieldOriented(shiftForwardRobotOriented).withName("Shift Forward");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftBackRobotOriented =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> -1 * DriveConstants.POV_SPEED, () -> 0.0)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftBack = drivebase.driveFieldOriented(shiftBackRobotOriented).withName("Shift Back");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftRightRobotOriented =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> 0.0, () -> -1 * DriveConstants.POV_SPEED)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftRight =
      drivebase.driveFieldOriented(shiftRightRobotOriented).withName("Shift Right");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftLeftRobotOriented =
      SwerveInputStream.of(drivebase.getSwerveDrive(), () -> 0.0, () -> DriveConstants.POV_SPEED)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftLeft = drivebase.driveFieldOriented(shiftLeftRobotOriented).withName("Shift Left");

  Command diamondDrive =
      drivebase.diamondDriveCommand(driveAngularVelocity).withName("Diamond Drive");

  Command aimHubDrive =
      new DeferredCommand(() -> game.aimHubDriveCommand(driveAngularVelocity), Set.of(drivebase))
          .withName("Aim Hub Drive");

  // Commands to drive to the closest spot that is at the ideal range to the hub and aim for launch.
  private Command driveToLaunchPosition =
      new DeferredCommand(game::createDriveLaunchCommand, Set.of(drivebase))
          .withName("Drive to Launch");
  private Command driveThroughLeftTrench =
      new DeferredCommand(() -> game.createDriveTrenchCommand(true), Set.of(drivebase))
          .withName("Drive Left Trench");
  private Command driveThroughRightTrench =
      new DeferredCommand(() -> game.createDriveTrenchCommand(false), Set.of(drivebase))
          .withName("Drive Right Trench");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Publish subsystem data including commands
    SmartDashboard.putData(drivebase);
    SmartDashboard.putData(ballSubsystem);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Commands for Autos
    NamedCommands.registerCommand("Launch 8", ballSubsystem.launchCommand().withTimeout(3.0));
    NamedCommands.registerCommand("Launch Full", ballSubsystem.launchCommand().withTimeout(5.0));
    NamedCommands.registerCommand("Intake", ballSubsystem.intakeCommand().withTimeout(10.0));

    // Setup the auto command chooser using the PathPlanner autos
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // ---------- Driver Controller ----------

    // Change drive type from field oriented to robot oriented, which is similar to tank drive, when
    // 'RB' is pressed on the driver's controller
    driverController.rightBumper().whileTrue(driveRobotOrientedAngularVelocity);

    // lock the wheels in a X pattern while left bumper is held
    driverController
        .leftBumper()
        .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    // adjust the robot to the closest 45 degree angle
    driverController.rightTrigger().whileTrue(diamondDrive);
    // adjust the robot to face the hub while driving
    driverController.leftTrigger().whileTrue(aimHubDrive);
    // Drive to a launch position near the hub when 'A' is pressed on the driver's controller
    driverController
        .a()
        .whileTrue(
            driveToLaunchPosition.andThen(
                ballSubsystem.launchCommand().withName("Launch after Drive to Hub")));
    driverController.b().whileTrue(driveThroughLeftTrench);
    driverController.x().whileTrue(driveThroughRightTrench);

    // Drives the robot slowly to a set position based on which of the pov buttons is pressed on the
    // driver's controller
    driverController.povUp().whileTrue(shiftForward);
    driverController.povDown().whileTrue(shiftBack);
    driverController.povRight().whileTrue(shiftRight);
    driverController.povLeft().whileTrue(shiftLeft);

    driverController
        .y()
        .whileTrue(
            shiftForward
                .alongWith(ballSubsystem.intakeCommand())
                .withName("Drive Forward and Intake"));

    // Zero the gyro when 'start' is pressed on the driver's controller
    driverController
        .start()
        .onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance).ignoringDisable(true));

    // ---------- Operator Controller ----------
    // Define operator commands and button mappings here

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(ballSubsystem.intakeCommand().withName("Intake"));

    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(ballSubsystem.launchCommand().withName("Launch"));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(ballSubsystem.ejectCommand().withName("Eject"));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {

    DataLogManager.log("disableSubsystems");
  }

  /**
   * Sets the motor brake mode for the drivebase.
   *
   * @param brake true to enable motor braking, false to set coast mode
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
    DataLogManager.log("Drive Brake: " + brake);
  }

  /**
   * Get the drive command from the drive subsystem.
   *
   * @return the Command for teleop driving or null
   */
  public Command getTeleopDriveCommand() {
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  /**
   * Set the LEDs to a specified color.
   *
   * @param pattern the LED pattern to set
   */
  public void setLeds(LEDPattern pattern) {
    led.setPattern(pattern);
  }

  /** Set the LEDs to show robot status. */
  public void setLedStatus() {
    LEDPattern desired;

    if (DriverStation.isDisabled()) {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Red) {
          desired = LEDPattern.solid(Color.kRed).breathe(Seconds.of(4.0));
        } else {
          desired = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(4.0));
        }
      } else {
        desired = LEDPattern.solid(Color.kGray);
      }
    } else {

      boolean hubActive = HubTracker.isActive();

      if (!hubActive) {
        desired = LEDPattern.solid(Color.kPurple);
      } else {
        desired = LEDPattern.solid(Color.kGreen);

        if (game.isRobotReadyAtHub()) {
          desired = LEDPattern.solid(Color.kOrange);
        }
      }

      var timeRemaining = HubTracker.timeRemainingInCurrentShift();
      double timeRemainingSeconds =
          timeRemaining.isPresent() ? timeRemaining.get().in(Seconds) : 0.0;

      if (timeRemainingSeconds < 5.0 && timeRemainingSeconds > 0.0) {
        desired = desired.blink(Seconds.of(0.5));
      }
    }

    led.setPattern(desired);
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the drive Subsystem.
   *
   * @return a reference to the drive Subsystem
   */
  public SwerveSubsystem getDriveSubsystem() {
    return drivebase;
  }

  /**
   * Use this to get the Ball Subsystem.
   *
   * @return a reference to the Ball Subsystem
   */
  public CANFuelSubsystem getBallSubsystem() {
    return ballSubsystem;
  }

  /** This should be called periodically from the main {@link Robot} class. */
  public void periodic() {
    game.periodic();
    setLedStatus();
  }
}
