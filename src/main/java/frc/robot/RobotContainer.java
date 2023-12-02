// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.helpers.Vision;
import frc.robot.subsystems.LightShow;

public class RobotContainer {

  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
  }

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);
  private final Vision m_visionTagOfDetectionAndGreatestIntelligence = new Vision();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_visionTagOfDetectionAndGreatestIntelligence.start();
    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
  }

  public void handleDisable() {
    // createAutonomousSelector();
  }

  private void configureButtonBindings() {

    // Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm);
    // Command moveToReceive = new RunCommand(() ->
    // m_arm.receiveFromSingleSubstation(-m_operatorController.getLeftY()), m_arm);
    // var leftBumper = new JoystickButton(m_driverController,
    // Button.kLeftBumper.value);
    // leftBumper.whileTrue(moveToReceive);
  }

  private void ensureSubsystemsHaveDefaultCommands() {
    // m_drivetrain.setDefaultCommand(fastDrive);
    // m_arm.setDefaultCommand(safelyRestTheArm);
    // m_intake.setDefaultCommand(stopRollers);
  }

  /**
   * This is called by the system when automomous runs, and it
   * should return the command you want to execute when automous
   * mode begins.
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

}