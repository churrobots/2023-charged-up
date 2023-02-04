// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.RawEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules

  private final BaseSwerveModule m_frontLeft = new AndymarkSwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPortsjr,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      (2.217730 / (2 * Math.PI)));

  private final BaseSwerveModule m_rearLeft = new AndymarkSwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPortsjr,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed,
      (1.422184 / (2 * Math.PI)));

  private final BaseSwerveModule m_frontRight = new AndymarkSwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPortsjr,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      (2.475519 / (2 * Math.PI)));

  private final BaseSwerveModule m_rearRight = new AndymarkSwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPortsjr,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed,
      (1.414083 / (2 * Math.PI)));

  // create the limiterkMaxAccelerationMetersPerSecondSquared
  private SlewRateLimiter swerveXAccelerationFilterLimiter = new SlewRateLimiter(
      Constants.DriveConstants.kMaxDriveAccelerationMetersPerSecondSquared);
  private SlewRateLimiter swerveYAccelerationFilterLimiter = new SlewRateLimiter(
      Constants.DriveConstants.kMaxDriveAccelerationMetersPerSecondSquared);

  // creates the inspector
  private final SubsystemInspector inspector = new SubsystemInspector("Drivetrain");

  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(5);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.ModuleConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.ModuleConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition() });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() });

    inspector.set("EncoderLeftFrontX", m_frontLeft.getPosition().angle.getRadians());
    inspector.set("EncoderLeftBackX", m_rearLeft.getPosition().angle.getRadians());
    inspector.set("EncoderRightFrontX", m_frontRight.getPosition().angle.getRadians());
    inspector.set("EncoderRightBackX", m_rearRight.getPosition().angle.getRadians());
    inspector.set("Gyro", getRotation2d().getRadians());

    inspector.set("EncoderLeftFrontXAbsValue", m_frontLeft.getPosition().toString());
    inspector.set("EncoderLeftBackXAbsValue", m_rearLeft.getPosition().toString());
    inspector.set("EncoderRightFrontXAbsValue", m_frontRight.getPosition().toString());
    inspector.set("EncoderRightBackXAbsValue", m_rearRight.getPosition().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // m_odometry.resetPosition(pose, getRotation2d());
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    if (rateLimit) {
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.ModuleConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void spinInPlace(Boolean counterclockwise) {
    double rotDelivered = counterclockwise ? 1 : -1;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotDelivered,
            Rotation2d.fromDegrees(m_gyro.getAngle())));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() % 360);
  }

  // var smoothedXThrottlePercent =
  // swerveXAccelerationFilterLimiter.calculate(xSpeed);
  // var smoothedYThrottlePercent =
  // swerveYAccelerationFilterLimiter.calculate(ySpeed);smoothedXThrottlePercent=MathUtil.applyDeadband(smoothedXThrottlePercent,Constants.joystickDeadband);smoothedYThrottlePercent=MathUtil.applyDeadband(smoothedYThrottlePercent,Constants.joystickDeadband);inspector.set("smoothedX",smoothedXThrottlePercent);inspector.set("smoothedY",smoothedYThrottlePercent);inspector.set("xSpeed",xSpeed);inspector.set("ySpeed",ySpeed);
  // var chassisSpeeds = fieldRelative
  // ? ChassisSpeeds.fromFieldRelativeSpeeds(smoothedXThrottlePercent,
  // smoothedYThrottlePercent, rot,
  // getRotation2d())
  // : new ChassisSpeeds(smoothedXThrottlePercent, smoothedYThrottlePercent, rot);

  // var swerveModuleStates =
  // DriveConstants.kDriveKinematics.toSwerveModuleStates(
  // chassisSpeeds);SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,DriveConstants.kMaxSpeedMetersPerSecond);m_frontLeft.setDesiredState(swerveModuleStates[0]);m_frontRight.setDesiredState(swerveModuleStates[1]);m_rearLeft.setDesiredState(swerveModuleStates[2]);m_rearRight.setDesiredState(swerveModuleStates[3]);
  // }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setModulePositions(SwerveModulePosition[] desiredPositions) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    // desiredPositions, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredPosition(desiredPositions[0]);
    m_frontRight.setDesiredPosition(desiredPositions[1]);
    m_rearLeft.setDesiredPosition(desiredPositions[2]);
    m_rearRight.setDesiredPosition(desiredPositions[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}