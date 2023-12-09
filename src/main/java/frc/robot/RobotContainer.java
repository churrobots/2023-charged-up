// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.helpers.Vision;
import frc.robot.subsystems.LightShow;

public class RobotContainer {

  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
  }

  // Inputs
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);
  private final Vision m_visionTagOfDetectionAndGreatestIntelligence = new Vision();

  // Outputs
  private final LightShow m_lightShow = new LightShow();

  public void bootup() {

    m_visionTagOfDetectionAndGreatestIntelligence.start();

    var showRed = new RunCommand(m_lightShow::setRed, m_lightShow);
    var showGreen = new RunCommand(m_lightShow::setGreen, m_lightShow);
    var showPurple = new RunCommand(m_lightShow::setPurple, m_lightShow);
    var showBlue = new RunCommand(m_lightShow::setBlue, m_lightShow);

    var showing1 = new Trigger(() -> {
      return m_visionTagOfDetectionAndGreatestIntelligence.getMostRecentAprilTag() == 1;
    });

    var showing2 = new Trigger(() -> {
      return m_visionTagOfDetectionAndGreatestIntelligence.getMostRecentAprilTag() == 2;
    });

    var showing3 = new Trigger(() -> {
      return m_visionTagOfDetectionAndGreatestIntelligence.getMostRecentAprilTag() == 3;
    });

    showing1.whileTrue(showRed);
    showing2.whileTrue(showGreen);
    showing3.whileTrue(showBlue);

    m_lightShow.setDefaultCommand(showPurple);
  }

  public void handleDisable() {
    m_lightShow.setPurple();
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

}