// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.swerve.AndymarkFalconSwerveModule;
import frc.robot.helpers.swerve.BaseSwerveModule;
import frc.robot.helpers.swerve.RevMAXSwerveModule;
import frc.robot.helpers.swerve.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final SubsystemInspector inspector = new SubsystemInspector(getSubsystem());

  private final BaseSwerveModule m_frontLeft;
  private final BaseSwerveModule m_frontRight;
  private final BaseSwerveModule m_rearLeft;
  private final BaseSwerveModule m_rearRight;
  public final WPI_Pigeon2 m_gyro;
  private final SwerveDriveKinematics m_kinematics;
  private final SlewRateLimiter m_magLimiter;
  private final SlewRateLimiter m_rotLimiter;
  private final double m_maxSpeedMetersPerSecond;
  private final double m_maxAngularSpeedRadiansPerSecond;
  private final double m_directionSlewRate;
  private final boolean m_gyroIsReversed;

  public static enum WhichDrivebase {
    TurboSwervo,
    SpeedyHedgehog
  }

  /**
   * Constants that work for the TurboSwervo drive base.
   * This includes the CAN configuration as well as dimensions,
   * max speeds, etc that are particular to TurboSwervo.
   */
  private static final class TurboSwervoConstants {

    // Chassis configuration
    // TODO: get actual numbers, measured center-to-center of the wheels (not frame)
    public static final double kTrackWidth = Units.inchesToMeters(18.25);
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    // Angular offsets of the modules relative to the chassis in radians
    // TODO: need to actually calculate these since the robot is not square
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // CAN bus configuration
    public static final int kFrontLeftDriveCanId = 3;
    public static final int kRearLeftDriveCanId = 1;
    public static final int kFrontRightDriveCanId = 2;
    public static final int kRearRightDriveCanId = 4;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 7;

    // Analog encoder configuration
    public static final int kFrontLeftTurningEncoderAnalogPort = 1;
    public static final int kRearLeftTurningEncoderAnalogPort = 3;
    public static final int kFrontRightTurningEncoderAnalogPort = 0;
    public static final int kRearRightTurningEncoderAnalogPort = 2;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    // Gyro config
    public static final int kGyroCanId = 9;
    public static final boolean kGyroReversed = false;

  }

  /**
   * Constants that work for the SpeedyHedgehog drive base.
   * This includes the CAN configuration as well as dimensions,
   * max speeds, etc that are particular to TurboSwervo.
   */
  private static final class SpeedyHedgehogConstants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    // Angular offsets of the modules relative to the chassis in radians
    // TODO: need to actually calculate these since the robot is not square
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kTurboMaxSpeedMetersPerSecond = 6;
    public static final double kTurboMaxAngularSpeed = 3 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 2;
    // TODO: we got this error one day, causes turning to "twitch" periodically
    // https://www.chiefdelphi.com/t/vmx-pi-can-spark-max-ids-1-timed-out-while-waiting-for-periodic-status-0/402177/8
    public static final int kRearRightTurningCanId = 4;

    // Gyro config
    public static final int kGyroCanId = 9;
    public static final boolean kGyroReversed = false;

    // PID contants
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  public DriveSubsystem(WhichDrivebase whichDrivebase) {

    if (whichDrivebase == WhichDrivebase.TurboSwervo) {

      /////////////////////////////////////////////////////////////////
      // TurboSwervo is a Falcon-based AndyMark Swerve and Steer

      m_frontLeft = new AndymarkFalconSwerveModule(
          "frontLeft",
          TurboSwervoConstants.kFrontLeftDriveCanId,
          TurboSwervoConstants.kFrontLeftTurningCanId,
          TurboSwervoConstants.kFrontLeftTurningEncoderAnalogPort,
          TurboSwervoConstants.kFrontLeftDriveEncoderReversed,
          TurboSwervoConstants.kFrontLeftTurningEncoderReversed,
          TurboSwervoConstants.kFrontLeftChassisAngularOffset);

      m_rearLeft = new AndymarkFalconSwerveModule(
          "rearLeft",
          TurboSwervoConstants.kRearLeftDriveCanId,
          TurboSwervoConstants.kRearLeftTurningCanId,
          TurboSwervoConstants.kRearLeftTurningEncoderAnalogPort,
          TurboSwervoConstants.kRearLeftDriveEncoderReversed,
          TurboSwervoConstants.kRearLeftTurningEncoderReversed,
          TurboSwervoConstants.kRearLeftChassisAngularOffset);

      m_frontRight = new AndymarkFalconSwerveModule(
          "frontRight",
          TurboSwervoConstants.kFrontRightDriveCanId,
          TurboSwervoConstants.kFrontRightTurningCanId,
          TurboSwervoConstants.kFrontRightTurningEncoderAnalogPort,
          TurboSwervoConstants.kFrontRightDriveEncoderReversed,
          TurboSwervoConstants.kFrontRightTurningEncoderReversed,
          TurboSwervoConstants.kFrontRightChassisAngularOffset);

      m_rearRight = new AndymarkFalconSwerveModule(
          "rearRight",
          TurboSwervoConstants.kRearRightDriveCanId,
          TurboSwervoConstants.kRearRightTurningCanId,
          TurboSwervoConstants.kRearRightTurningEncoderAnalogPort,
          TurboSwervoConstants.kRearRightDriveEncoderReversed,
          TurboSwervoConstants.kRearRightTurningEncoderReversed,
          TurboSwervoConstants.kRearRightChassisAngularOffset);

      m_kinematics = new SwerveDriveKinematics(
          new Translation2d(TurboSwervoConstants.kWheelBase / 2, TurboSwervoConstants.kTrackWidth / 2),
          new Translation2d(TurboSwervoConstants.kWheelBase / 2, -TurboSwervoConstants.kTrackWidth / 2),
          new Translation2d(-TurboSwervoConstants.kWheelBase / 2, TurboSwervoConstants.kTrackWidth / 2),
          new Translation2d(-TurboSwervoConstants.kWheelBase / 2, -TurboSwervoConstants.kTrackWidth / 2));

      m_gyro = new WPI_Pigeon2(TurboSwervoConstants.kGyroCanId);
      m_gyroIsReversed = TurboSwervoConstants.kGyroReversed;

      m_magLimiter = new SlewRateLimiter(TurboSwervoConstants.kMagnitudeSlewRate);
      m_rotLimiter = new SlewRateLimiter(TurboSwervoConstants.kRotationalSlewRate);

      m_directionSlewRate = TurboSwervoConstants.kDirectionSlewRate;
      m_maxAngularSpeedRadiansPerSecond = TurboSwervoConstants.kMaxAngularSpeed;
      m_maxSpeedMetersPerSecond = TurboSwervoConstants.kMaxSpeedMetersPerSecond;

    } else {

      /////////////////////////////////////////////////////////////////
      // SpeedyHedgehog is a Rev MAXSwerve

      m_frontLeft = new RevMAXSwerveModule(
          SpeedyHedgehogConstants.kFrontLeftDrivingCanId,
          SpeedyHedgehogConstants.kFrontLeftTurningCanId,
          SpeedyHedgehogConstants.kFrontLeftChassisAngularOffset);

      m_frontRight = new RevMAXSwerveModule(
          SpeedyHedgehogConstants.kFrontRightDrivingCanId,
          SpeedyHedgehogConstants.kFrontRightTurningCanId,
          SpeedyHedgehogConstants.kFrontRightChassisAngularOffset);

      m_rearLeft = new RevMAXSwerveModule(
          SpeedyHedgehogConstants.kRearLeftDrivingCanId,
          SpeedyHedgehogConstants.kRearLeftTurningCanId,
          SpeedyHedgehogConstants.kRearLeftChassisAngularOffset);

      m_rearRight = new RevMAXSwerveModule(
          SpeedyHedgehogConstants.kRearRightDrivingCanId,
          SpeedyHedgehogConstants.kRearRightTurningCanId,
          SpeedyHedgehogConstants.kRearRightChassisAngularOffset);

      m_kinematics = new SwerveDriveKinematics(
          new Translation2d(SpeedyHedgehogConstants.kWheelBase / 2, SpeedyHedgehogConstants.kTrackWidth / 2),
          new Translation2d(SpeedyHedgehogConstants.kWheelBase / 2, -SpeedyHedgehogConstants.kTrackWidth / 2),
          new Translation2d(-SpeedyHedgehogConstants.kWheelBase / 2, SpeedyHedgehogConstants.kTrackWidth / 2),
          new Translation2d(-SpeedyHedgehogConstants.kWheelBase / 2, -SpeedyHedgehogConstants.kTrackWidth / 2));

      m_gyro = new WPI_Pigeon2(SpeedyHedgehogConstants.kGyroCanId);
      m_gyroIsReversed = SpeedyHedgehogConstants.kGyroReversed;

      m_magLimiter = new SlewRateLimiter(SpeedyHedgehogConstants.kMagnitudeSlewRate);
      m_rotLimiter = new SlewRateLimiter(SpeedyHedgehogConstants.kRotationalSlewRate);

      m_directionSlewRate = SpeedyHedgehogConstants.kDirectionSlewRate;
      m_maxAngularSpeedRadiansPerSecond = SpeedyHedgehogConstants.kMaxAngularSpeed;
      m_maxSpeedMetersPerSecond = SpeedyHedgehogConstants.kMaxSpeedMetersPerSecond;
    }

    // Slew rate filter variables for controlling lateral acceleration
    m_currentRotation = 0.0;
    m_currentTranslationDir = 0.0;
    m_currentTranslationMag = 0.0;

    m_prevTime = WPIUtilJNI.now() * 1e-6;
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  @Override
  public void periodic() {

    inspector.set("gyro.yaw", m_gyro.getYaw());
    inspector.set("gyro.pitch", m_gyro.getPitch());
    inspector.set("gyro.roll", m_gyro.getRoll());
    inspector.set("gyro.angle", m_gyro.getAngle());

    // Update the odometry in the periodic block
    m_odometry.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
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
    m_odometry.resetPosition(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Resets the gyro as if the robot were facing away from you.
   * This is helpful for resetting field-oriented driving.
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(m_directionSlewRate / m_currentTranslationMag);
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
    double xSpeedDelivered = xSpeedCommanded * m_maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * m_maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * m_maxAngularSpeedRadiansPerSecond;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getGyroAngle())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, m_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetDriveEncodersToZero();
    m_frontRight.resetDriveEncodersToZero();
    m_rearLeft.resetDriveEncodersToZero();
    m_rearRight.resetDriveEncodersToZero();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() % 360 * -1);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroAngle().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (m_gyroIsReversed ? -1.0 : 1.0);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void assertWheelsArePointedForwardAndStoreCalibration() {
    m_frontLeft.assertModuleIsPointedForwardAndStoreCalibration();
    m_frontRight.assertModuleIsPointedForwardAndStoreCalibration();
    m_rearLeft.assertModuleIsPointedForwardAndStoreCalibration();
    m_rearRight.assertModuleIsPointedForwardAndStoreCalibration();
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),

        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.m_kinematics, // SwerveDriveKinematics
            new PIDController(SpeedyHedgehogConstants.kPXController, 0, 0), // X controller. Tune these values for your
                                                                            // robot. Leaving them 0 will only use
            // feedforwards.
            new PIDController(SpeedyHedgehogConstants.kPYController, 0, 0), // Y controller (usually the same values as
                                                                            // X controller)
            new PIDController(SpeedyHedgehogConstants.kPThetaController, 0, 0), // Rotation controller. Tune these
                                                                                // values for your
            // robot. Leaving them 0 will
            // only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            this // Requires this drive subsystem
        ));
  }

}