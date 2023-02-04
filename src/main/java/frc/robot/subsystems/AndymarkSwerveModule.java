// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class AndymarkSwerveModule extends BaseSwerveModule {
  private final WPI_TalonFX driveMotor;
  private final WPI_VictorSPX m_turningMotor;

  private final AnalogEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderChannels   The channels of the drive encoder.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public AndymarkSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double turningOffset) {

    driveMotor = new WPI_TalonFX(driveMotorChannel);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.2));
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));

    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);

    m_turningEncoder = new AnalogEncoder(turningEncoderChannel);
    m_turningEncoder.setPositionOffset(turningOffset);
    m_turningEncoder.setDistancePerRotation(2 * Math.PI);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not

    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.

    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not

    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  private double convertSensorCountsToDistanceInMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.sensorUnitsPerRevolution;
    double wheelRotations = motorRotations / Constants.driveGearRatio;
    double inchesOfRotation = wheelRotations * 2 * Math.PI * Constants.driveWheelRadiusInInches;
    return Units.inchesToMeters(inchesOfRotation);
  }

  private double getTurningRadians() {
    return m_turningEncoder.getDistance();
  }

  // private double getAbsPostition() {
  // return m_turningEncoder.getAbsolutePosition();
  // }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  // @Override
  // public SwerveModuleState getState() {
  // return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new
  // Rotation2d(getTurningRadians()));
  // }

  // @Override
  // public SwerveModulePosition getPosition() {
  // return new
  // SwerveModulePosition(convertSensorCountsToDistanceInMeters(driveMotor.getSelectedSensorVelocity()),
  // new Rotation2d(getTurningRadians()));
  // }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        falconToMPS(driveMotor.getSelectedSensorVelocity(),
            Constants.ModuleConstants.kWheelCircumferenceMeters,
            Constants.driveGearRatio),
        new Rotation2d(getTurningRadians()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        falconToMeters(driveMotor.getSelectedSensorPosition(),
            Constants.ModuleConstants.kWheelCircumferenceMeters,
            Constants.driveGearRatio),
        new Rotation2d(getTurningRadians()));
  }

  public static double falconToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * 2048.0));
  }

  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
    return positionCounts * (circumference / (gearRatio * 2048.0));
  }

  @Override
  public Rotation2d getAngle() {
    return getState().angle;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
        new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    double actualMetersPerSecond = driveMotor.getSelectedSensorVelocity();
    actualMetersPerSecond = convertSensorCountsToDistanceInMeters(actualMetersPerSecond);
    final double driveOutput = m_drivePIDController.calculate(actualMetersPerSecond,
        optimizedDesiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurningRadians(),
        optimizedDesiredState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  @Override
  public void setDesiredPosition(SwerveModulePosition desiredPosition) {
    // // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState optimizedDesiredState =
    // SwerveModulePosition.optimize(desiredPosition,
    // new Rotation2d(getTurningRadians()));

    // // Calculate the drive output from the drive PID controller.
    // double actualMetersPerSecond = driveMotor.getSelectedSensorVelocity();
    // actualMetersPerSecond =
    // convertSensorCountsToDistanceInMeters(actualMetersPerSecond);
    // final double driveOutput =
    // m_drivePIDController.calculate(actualMetersPerSecond,
    // optimizedDesiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurningRadians(),
        desiredPosition.angle.getRadians());

    // // Calculate the turning motor output from the turning PID controller.
    // driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  @Override
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
  }

}