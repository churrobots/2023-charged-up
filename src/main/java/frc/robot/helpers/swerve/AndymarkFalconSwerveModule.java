// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AndymarkFalconSwerveModule extends BaseSwerveModule {
  private final String moduleIdentifier;
  private final WPI_TalonFX driveMotor;
  private final WPI_VictorSPX m_turningMotor;

  private final AnalogEncoder m_turningEncoder;

  private final double kPModuleDriveController = 1;
  private final double kPModuleTurningController = 1;
  private final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
  private final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
  private final double driveGearRatio = 6.67;
  private final double driveWheelRadiusInInches = 2;

  // The Falcon 500s have a Talon FX Integrated sensor, which is rated for 2048
  // units per rotation:
  // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
  private final int sensorUnitsPerRevolution = 2048;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kMaxModuleAngularSpeedRadiansPerSecond,
          kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // TODO: Gains are for example purposes only
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public AndymarkFalconSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {

    this.moduleIdentifier = "turningModule" + String.valueOf(turningEncoderChannel);

    // Drive motor configuration
    driveMotor = new WPI_TalonFX(driveMotorChannel);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.2));
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));

    // Turning motor configuration
    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel);
    m_turningEncoder.setPositionOffset(_getTurningOffsetEntry().getDouble(0));
    m_turningEncoder.setDistancePerRotation(2 * Math.PI);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getDriveEncoderMetersPerSecond() {
    return convertSensorCountsToDistanceInMeters(driveMotor.getSelectedSensorVelocity());
  }

  private double getDriveEncoderDistanceInMeters() {
    return convertSensorCountsToDistanceInMeters(driveMotor.getSelectedSensorPosition());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  private double convertSensorCountsToDistanceInMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / sensorUnitsPerRevolution;
    double wheelRotations = motorRotations / driveGearRatio;
    double inchesOfRotation = wheelRotations * 2 * Math.PI * driveWheelRadiusInInches;
    return Units.inchesToMeters(inchesOfRotation);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderMetersPerSecond(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderDistanceInMeters(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getDriveEncoderMetersPerSecond(),
        state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
        state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  /** Zeroes all the SwerveModule encoders. */
  @Override
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
  }

  private NetworkTableEntry _getTurningOffsetEntry() {
    return SmartDashboard.getEntry("AndymarkSwerveModule.offset." + moduleIdentifier);
  }

  public void assertModuleIsPointedForwardAndStoreCalibration() {
    var entry = _getTurningOffsetEntry();
    var offset = m_turningEncoder.getAbsolutePosition();
    entry.setDouble(offset);
    entry.setPersistent();
    m_turningEncoder.setPositionOffset(offset);
  }

}
