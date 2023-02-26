// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.FalconHelper;
import frc.robot.helpers.SubsystemInspector;

public class Arm2 extends SubsystemBase {
  /** Creates a new Arm2. */
  private static final class Constants {
    private static final int armCanID = 12;
  }

  private final SubsystemInspector m_inspector = new SubsystemInspector(getSubsystem());
  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm2() {
    armMotor.configFactoryDefault();
    var safeCurrentLimitsForFalcon = new StatorCurrentLimitConfiguration(true, 35, 40, 2.5);
    armMotor.configStatorCurrentLimit(safeCurrentLimitsForFalcon);
    armMotor.setNeutralMode(NeutralMode.Brake);
    FalconHelper.configureMotionMagic(
        armMotor,
        18000,
        8000,
        3,
        0.2,
        0.2,
        0,
        0);
  }

  public void manuallyCalibrate() {
    armMotor.set(TalonFXControlMode.PercentOutput, -.10);
    armMotor.setSelectedSensorPosition(0);
    m_isCalibrated = true;
  }

  public void moveUp() {
    armMotor.set(TalonFXControlMode.PercentOutput, -.15);
  }

  public void moveDown() {
    armMotor.set(TalonFXControlMode.PercentOutput, .15);
  }

  public void receiveFromSingleSubstation() {
    armMotor.set(TalonFXControlMode.MotionMagic, 10000);
  }

  public void stop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
