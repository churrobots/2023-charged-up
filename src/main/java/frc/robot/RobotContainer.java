/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.helpers.Gamepad;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Connect to all the inputs (gamepads and shuffleboard).
  Gamepad driverGamepad = new Gamepad(Constants.driverGamepadPort);
  Gamepad operatorGamepad = new Gamepad(Constants.operatorGamepadPort);

  SwerveDrivetrain m_robotDrive = new SwerveDrivetrain();
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Create the autonomous chooser.
  SendableChooser<Command> autonomousChooser = new SendableChooser<Command>();

  public RobotContainer() {

    // Enable the camera.
    CameraServer.startAutomaticCapture();

    // Calibrate (uncomment this and deploy to a calibrated robot)
    // m_robotDrive.calibrateAllModules();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                // These are intentionally inverted and reversed yay
                -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
                MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
                -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1),
                true),
            m_robotDrive));

    // autonomousChooser.setDefaultOption("1-ball Auto: Drive, Wait, Dump",
    // waitForTeammate);
    // autonomousChooser.addOption("2-ball Auto: From WALL Tarmac",
    // twoBallAutoWallTarmac);
    SmartDashboard.putData(autonomousChooser);

    // Show all the subsystems in the smartdashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  public Command getAutonomousCommand() {
    // var thetaController =
    // new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    // new SwerveControllerCommand(
    // exampleTrajectory,
    // m_robotDrive::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,
    // ts.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false));
    return autonomousChooser.getSelected();
  }

  public Trajectory getTrajectory(String trajectoryName) {

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(
        "pathplanner/generatedJSON/" + trajectoryName + ".wpilib.json");

    try {
      var preprogrammedTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return preprogrammedTrajectory;
    } catch (IOException err) {
      DriverStation.reportError("broken!!!", err.getStackTrace());
      var emptyTrajectory = new Trajectory();
      return emptyTrajectory;
    }

  }

}
