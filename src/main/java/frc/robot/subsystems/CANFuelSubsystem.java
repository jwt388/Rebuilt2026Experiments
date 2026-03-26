// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Game;
import frc.robot.util.TunableNumber;

/**
 * Subsystem to control the fuel mechanisms on the robot including the intake, feeder and launcher.
 */
public class CANFuelSubsystem extends SubsystemBase {
  private final Game game;
  private final SparkMax feederRoller;
  private final SparkMax launcherRoller;
  private final SparkMax launcherRoller2;
  private final SparkMax intakeRoller;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder launcherEncoder;
  private final RelativeEncoder launcherEncoder2;
  private final RelativeEncoder intakeEncoder;
  private SparkClosedLoopController launcherController;
  private SparkClosedLoopController launcherController2;

  private SlewRateLimiter limiter;
  private double feederGoal = 0.0;
  private double launcherGoal = 0.0;
  private boolean launcherEnabled = false;
  private double intakeGoal = 0.0;

  // Setup tunable numbers and controllers for the motor.
  private TunableNumber intakingFeederVoltage =
      new TunableNumber("Intaking feeder roller value", FuelConstants.INTAKING_FEEDER_VOLTAGE);
  private TunableNumber intakingIntakeVoltage =
      new TunableNumber("Intaking intake roller value", FuelConstants.INTAKING_INTAKE_VOLTAGE);
  private TunableNumber launchingFeederVoltage =
      new TunableNumber("Launching feeder roller value", FuelConstants.LAUNCHING_FEEDER_VOLTAGE);
  private TunableNumber launchingIntakeVoltage =
      new TunableNumber("Launching intake roller value", FuelConstants.LAUNCHING_INTAKE_VOLTAGE);
  private TunableNumber spinUpFeederVoltage =
      new TunableNumber("Spin-up feeder roller value", FuelConstants.SPIN_UP_FEEDER_VOLTAGE);
  private TunableNumber ejectingFeederVoltage =
      new TunableNumber("Ejecting feeder roller value", FuelConstants.EJECTING_FEEDER_VOLTAGE);
  private TunableNumber ejectingIntakeVoltage =
      new TunableNumber("Ejecting intake roller value", FuelConstants.EJECTING_INTAKE_VOLTAGE);
  private TunableNumber proportionalGain =
      new TunableNumber("Launcher Kp", FuelConstants.LAUNCHER_KP_VOLTS_PER_RPM);
  private TunableNumber derivativeGain =
      new TunableNumber("Launcher Kd", FuelConstants.LAUNCHER_KD_VOLTS_PER_RPM_SEC);
  private TunableNumber velocityGain =
      new TunableNumber("Launcher Kv", FuelConstants.LAUNCHER_KV_VOLTS_PER_RPM);
  private TunableNumber launcherRpm =
      new TunableNumber("Launcher Speed RPM", FuelConstants.LAUNCHER_SPEED_RPM);
  private TunableNumber enableLaunchTable =
      new TunableNumber("Launcher Enable Table", FuelConstants.ENABLE_LAUNCH_TABLE);
  private TunableNumber launchSpinupThreshold =
      new TunableNumber("Launcher Spin up", FuelConstants.LAUNCH_SPINUP_THRESHOLD);

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem(Game game) {
    this.game = game;

    limiter = new SlewRateLimiter(FuelConstants.RATE_LIMIT);

    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new SparkMax(FuelConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    launcherRoller2 = new SparkMax(FuelConstants.LAUNCHER_MOTOR2_ID, MotorType.kBrushless);
    intakeRoller = new SparkMax(FuelConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederEncoder = feederRoller.getEncoder();
    launcherEncoder = launcherRoller.getEncoder();
    launcherEncoder2 = launcherRoller2.getEncoder();
    intakeEncoder = intakeRoller.getEncoder();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.idleMode(IdleMode.kBrake);
    launcherConfig.smartCurrentLimit(FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT);

    launcherConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);

    // Closed loop controller on SparkMax
    launcherController = launcherRoller.getClosedLoopController();
    launcherController2 = launcherRoller2.getClosedLoopController();
    launcherConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(proportionalGain.get())
        .i(0)
        .d(derivativeGain.get())
        .outputRange(-1.0, 1.0)
        .feedForward
        .kV(velocityGain.get());
    launcherRoller.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherRoller2.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the intake roller, set a current limit, set
    // the motor to inverted so that positive values are used for intaking,
    // and apply the config to the controller
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(FuelConstants.INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** A method to set the rollers to values for intaking. */
  public void intake() {
    feederGoal = intakingFeederVoltage.get();
    launcherGoal = 0.0;
    launcherController.setSetpoint(launcherGoal, ControlType.kVelocity);
    intakeGoal = intakingIntakeVoltage.get();
  }

  /** A method to set the rollers to values for ejecting fuel out the intake. */
  public void eject() {
    feederGoal = ejectingFeederVoltage.get();
    launcherGoal = 0.0;
    launcherController.setSetpoint(launcherGoal, ControlType.kVelocity);
    intakeGoal = ejectingIntakeVoltage.get();
  }

  /**
   * A method to set the rollers to values for launching. Finds the needed launcher speed based on
   * the distance from the robot to the hub Feeder voltage is set automatically in the periodic
   * function based on whether the launcher is up to speed or not to help with launch consistency.
   */
  public void launch() {
    loadPidfTunableNumbers();
    launcherEnabled = true;
    if (enableLaunchTable.get() > 0.0) {
      launcherGoal = FuelConstants.LAUNCH_TABLE.get(game.getDistanceToHub())[1];
    } else {
      launcherGoal = launcherRpm.get();
    }
    launcherController.setSetpoint(launcherGoal, ControlType.kVelocity);
    launcherController2.setSetpoint(launcherGoal, ControlType.kVelocity);
    intakeGoal = launchingIntakeVoltage.get();
  }

  /** A method to stop the rollers. */
  public void stop() {
    launcherEnabled = false;
    feederGoal = 0.0;
    launcherGoal = 0.0;
    launcherController.setSetpoint(0.0, ControlType.kVoltage);
    launcherController2.setSetpoint(0.0, ControlType.kVoltage);
    intakeGoal = 0.0;
  }

  /** A command factory to turn the launch method into a command that requires this subsystem. */
  public Command launchCommand() {
    return this.runEnd(this::launch, this::stop);
  }

  /** A command factory to turn the eject method into a command that requires this subsystem. */
  public Command ejectCommand() {
    return this.runEnd(this::eject, this::stop);
  }

  /** A command factory to turn the intake method into a command that requires this subsystem. */
  public Command intakeCommand() {
    return this.runEnd(this::intake, this::stop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If the launcher is running but not up to speed, run the feeder inward. When the
    // launcher is up to speed, run the feeder to push balls into the launcher.
    if (launcherEnabled) {
      if (launcherEncoder.getVelocity() < launcherGoal * launchSpinupThreshold.get()) {
        feederGoal = spinUpFeederVoltage.get();
      } else {
        feederGoal = launchingFeederVoltage.get();
      }
    }

    // Use the slew rate limiter to ramp the feeder voltage to avoid sudden changes
    double feederVoltage = limiter.calculate(feederGoal);
    feederRoller.setVoltage(feederVoltage);
    intakeRoller.setVoltage(intakeGoal);

    // Update SmartDashboard values for monitoring
    SmartDashboard.putNumber("Feeder/Goal", feederGoal);
    SmartDashboard.putNumber("Feeder/Set Voltage", feederVoltage);
    SmartDashboard.putNumber("Launcher/Goal", launcherGoal);
    SmartDashboard.putNumber("Intake/Goal", intakeGoal);

    SmartDashboard.putNumber("Launcher/Current", launcherRoller.getOutputCurrent());
    SmartDashboard.putNumber("Launcher2/Current", launcherRoller2.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Current", intakeRoller.getOutputCurrent());
    SmartDashboard.putNumber(
        "Launcher/Voltage", launcherRoller.getAppliedOutput() * launcherRoller.getBusVoltage());
    SmartDashboard.putNumber(
        "Launcher2/Voltage", launcherRoller2.getAppliedOutput() * launcherRoller2.getBusVoltage());
    SmartDashboard.putNumber(
        "Intake/Voltage", intakeRoller.getAppliedOutput() * intakeRoller.getBusVoltage());
    SmartDashboard.putNumber("Launcher/Velocity", launcherEncoder.getVelocity());
    SmartDashboard.putNumber("Launcher2/Velocity", launcherEncoder2.getVelocity());
    SmartDashboard.putNumber("Intake/Velocity", intakeEncoder.getVelocity());

    SmartDashboard.putNumber("Feeder/Current", feederRoller.getOutputCurrent());
    SmartDashboard.putNumber(
        "Feeder/Voltage", feederRoller.getAppliedOutput() * feederRoller.getBusVoltage());
    SmartDashboard.putNumber("Feeder/Velocity", feederEncoder.getVelocity());
    SmartDashboard.putBoolean("Launcher/Enabled", launcherEnabled);

    SmartDashboard.putNumber("Launcher/MotorTemp", launcherRoller.getMotorTemperature());
    SmartDashboard.putNumber("Launcher2/MotorTemp", launcherRoller2.getMotorTemperature());
    SmartDashboard.putNumber("Feeder/MotorTemp", feederRoller.getMotorTemperature());
    SmartDashboard.putNumber("Intake/MotorTemp", intakeRoller.getMotorTemperature());

    SmartDashboard.putNumber("Launcher/BusVoltage", launcherRoller.getBusVoltage());
    SmartDashboard.putNumber("Launcher2/BusVoltage", launcherRoller2.getBusVoltage());
    SmartDashboard.putNumber("Feeder/BusVoltage", feederRoller.getBusVoltage());
    SmartDashboard.putNumber("Intake/BusVoltage", intakeRoller.getBusVoltage());
  }

  /**
   * Load PIDF values that can be tuned at runtime. This should only be called when the controller
   * is disabled - for example from enable().
   */
  private void loadPidfTunableNumbers() {

    // Read tunable values for PID controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig
        .closedLoop
        .p(proportionalGain.get())
        .d(derivativeGain.get())
        .feedForward
        .kV(velocityGain.get());
    launcherRoller.configure(
        launcherConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    launcherRoller2.configure(
        launcherConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Functions for simulation purposes

  /** Returns the feeder motor for simulation. */
  public SparkMax getFeederMotor() {
    return feederRoller;
  }

  /** Returns the intake motor for simulation. */
  public SparkMax getIntakeMotor() {
    return intakeRoller;
  }

  /** Returns the launcher motor for simulation. */
  public SparkMax getLauncherMotor() {
    return launcherRoller;
  }

  public double getLauncherVelocity() {
    return launcherEncoder.getVelocity();
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }
}
