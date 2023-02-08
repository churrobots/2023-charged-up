// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.SubsystemInspector;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  private final BaseSwerveModule m_frontLeft;
  private final BaseSwerveModule m_frontRight;
  private final BaseSwerveModule m_backLeft;
  private final BaseSwerveModule m_backRight;

  private final Gyro m_gyro;

  private final SwerveDriveKinematics m_kinematics;

  private final SwerveDriveOdometry m_odometry;

  private final SubsystemInspector inspector = new SubsystemInspector(getSubsystem());

  public SwerveDrivetrain() {

    // TODO: switch between the 2 robot drivetrains

    // BEGIN TURBOSWERVO
    /////////////////////////
    m_frontLeft = new AndymarkSwerveModuleV2(
        "frontLeft",
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningEncoderPortsjr,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed);

    m_backLeft = new AndymarkSwerveModuleV2(
        "backLeft",
        DriveConstants.kRearLeftDriveMotorPort,
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftTurningEncoderPortsjr,
        DriveConstants.kRearLeftDriveEncoderReversed,
        DriveConstants.kRearLeftTurningEncoderReversed);

    m_frontRight = new AndymarkSwerveModuleV2(
        "frontRight",
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningEncoderPortsjr,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed);

    m_backRight = new AndymarkSwerveModuleV2(
        "backRight",
        DriveConstants.kRearRightDriveMotorPort,
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightTurningEncoderPortsjr,
        DriveConstants.kRearRightDriveEncoderReversed,
        DriveConstants.kRearRightTurningEncoderReversed);

    var widthOffsetInMeters = 0.47 / 2;
    var lengthOffsetInMeters = 0.64 / 2;
    m_frontLeftLocation = new Translation2d(widthOffsetInMeters, lengthOffsetInMeters);
    m_frontRightLocation = new Translation2d(widthOffsetInMeters, -lengthOffsetInMeters);
    m_backLeftLocation = new Translation2d(-widthOffsetInMeters, lengthOffsetInMeters);
    m_backRightLocation = new Translation2d(-widthOffsetInMeters, -lengthOffsetInMeters);

    m_gyro = new WPI_Pigeon2(5);
    /////////////////////////
    // END TURBOSWERVO

    m_gyro.reset();
    m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    inspector.set("xSpeed", xSpeed);
    inspector.set("ySpeed", ySpeed);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public void calibrateAllModules() {
    inspector.set("wut", Math.random());
    m_frontLeft.calibrateTurningMotor();
    m_frontRight.calibrateTurningMotor();
    m_backLeft.calibrateTurningMotor();
    m_backRight.calibrateTurningMotor();
  }

  @Override
  public void periodic() {
    inspector.set("frontLeft.getRotations", m_frontLeft.getPosition().angle.getRotations());
    inspector.set("frontRight.getRotations", m_frontRight.getPosition().angle.getRotations());
    inspector.set("backLeft.getRotations", m_backLeft.getPosition().angle.getRotations());
    inspector.set("backRight.getRotations", m_backRight.getPosition().angle.getRotations());
  }
}