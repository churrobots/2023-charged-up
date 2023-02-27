// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AngleSnap;
import frc.robot.commands.JengaBalance;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RollBoth;
import frc.robot.commands.RollSingle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.DriveSubsystem.WhichDrivebase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(WhichDrivebase.SpeedyHedgehog);
  // Arm ChaosArm = new Arm();

  Arm2 armTheSecond = new Arm2();
  Intake2 intakeTheSecond = new Intake2();

  // private final Intake topChiliDogGrab = new Intake(10, true);
  // private final Intake bottomChiliDogGrab = new Intake(11, false);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);

  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;

    public static final double kDriveDeadband = 0.1;
  }

  private static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // TODO: go back to using our Gamepad helper for easier button code
    configureButtonBindings();

    // Configure drivetrain
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    var stopArm = new RunCommand(armTheSecond::stop, armTheSecond);
    armTheSecond.setDefaultCommand(stopArm);

    var stopRollers = new RunCommand(intakeTheSecond::stopThePlan, intakeTheSecond);
    intakeTheSecond.setDefaultCommand(stopRollers);

    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(armTheSecond);
    SmartDashboard.putData(intakeTheSecond);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    Command turnButtonY = new AngleSnap(0, m_robotDrive);
    Command turnButtonB = new AngleSnap(-90, m_robotDrive);
    Command turnButtonA = new AngleSnap(180, m_robotDrive);
    Command turnButtonX = new AngleSnap(90, m_robotDrive);
    Command setBalance = new JengaBalance(m_robotDrive, m_robotDrive.m_gyro);

    Command anchorInPlace = new RunCommand(() -> m_robotDrive.setX(), m_robotDrive);
    Command resetGyro = new RunCommand(() -> m_robotDrive.resetGyro(), m_robotDrive);

    Command yeet = new RunCommand(intakeTheSecond::yeetTheCubes, intakeTheSecond);
    Command yoink = new RunCommand(intakeTheSecond::yoinkTheCubes, intakeTheSecond);

    Command moveArmIntoCalibration = new RunCommand(armTheSecond::moveIntoCalibrationPosition, armTheSecond);
    Command resetArmCalibration = new RunCommand(armTheSecond::resetCalibration, armTheSecond);
    Command receiveFromSingleSubstation = new RunCommand(armTheSecond::receiveFromSingleSubstation, armTheSecond);

    // Command rollBoth = new RollBoth(ChaosArm, bottomChiliDogGrab,
    // topChiliDogGrab);
    // Command rollTop = new RollSingle(ChaosArm, topChiliDogGrab);
    // Command rollBottom = new RollSingle(ChaosArm, bottomChiliDogGrab);
    // Command moveArm = new MoveArm(ChaosArm, m_operatorController);

    var startButton = new JoystickButton(m_driverController, Button.kStart.value);
    var backButton = new JoystickButton(m_driverController, Button.kBack.value);
    var leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

    // Driver
    var aButton = new JoystickButton(m_driverController, Button.kA.value);
    var bButton = new JoystickButton(m_driverController, Button.kB.value);
    var yButton = new JoystickButton(m_driverController, Button.kY.value);
    var xButton = new JoystickButton(m_driverController, Button.kX.value);

    aButton.whileTrue(turnButtonA);
    bButton.whileTrue(turnButtonB);
    yButton.whileTrue(turnButtonY);
    xButton.whileTrue(turnButtonX);
    backButton.whileTrue(setBalance);

    leftBumper.whileTrue(anchorInPlace);
    rightBumper.whileTrue(anchorInPlace);
    startButton.whileTrue(resetGyro);

    // Operator
    var leftBumperOpButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    var rightShoulderOpButton = new JoystickButton(m_operatorController, Button.kRightBumper.value);
    var aOpButton = new JoystickButton(m_operatorController, Button.kA.value);
    var xOpButton = new JoystickButton(m_operatorController, Button.kX.value);
    var startOpButton = new JoystickButton(m_operatorController, Button.kStart.value);
    var backOpButton = new JoystickButton(m_operatorController, Button.kBack.value);

    leftBumperOpButton.whileTrue(receiveFromSingleSubstation);
    xOpButton.whileTrue(yoink);
    aOpButton.whileTrue(yeet);
    backOpButton.whileTrue(moveArmIntoCalibration);
    startOpButton.whileTrue(resetArmCalibration);

    // aOpButton.whileTrue(rollTop);
    // bOpButton.whileTrue(rollBottom);
    // yOpButton.whileTrue(rollBoth);
    // xOpButton.whileTrue(moveArm);
  }

  public Trajectory getTrajectory(String trajectoryJSON) {

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory trajectory = new Trajectory();
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException err) {
      DriverStation.reportError("broken!!!", err.getStackTrace());
    }
    return trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_robotDrive.getKinematics());

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory moveToBall = getTrajectory("pathplanner/generatedJSON/AfterTheDust.wpilib.json");

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        moveToBall,
        m_robotDrive::getPose, // Functional interface to feed supplier
        m_robotDrive.getKinematics(),

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(moveToBall.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

}