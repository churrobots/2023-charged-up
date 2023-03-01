// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.FalconHelper;

public class Arm2 extends SubsystemBase {

  private static final class Constants {
    private static final int armCanID = 12;
    private static final double calibrationVelocitySensorUnitsPerSecond = -1000;
  }

  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm2() {
    armMotor.configFactoryDefault();
    var safeCurrentLimitsForFalcon = new StatorCurrentLimitConfiguration(true, 40, 45, 2.5);
    armMotor.configStatorCurrentLimit(safeCurrentLimitsForFalcon);
    armMotor.setNeutralMode(NeutralMode.Brake);
    FalconHelper.configureMotionMagic(
        armMotor,
        18000,
        8000,
        1,
        0.2,
        0.2,
        0,
        0);
  }

  private void runMotorWithSafety(TalonFXControlMode mode, double value) {
    if (m_isCalibrated) {
      armMotor.set(mode, value);
    }
  }

  public void resetCalibration() {
    armMotor.setSelectedSensorPosition(0);
    m_isCalibrated = true;
  }

  public void moveIntoCalibrationPosition() {
    m_isCalibrated = false;
    armMotor.set(TalonFXControlMode.Velocity, Constants.calibrationVelocitySensorUnitsPerSecond);
  }

  public void moveUp() {
    runMotorWithSafety(TalonFXControlMode.PercentOutput, -.15);
  }

  public void moveDown() {
    runMotorWithSafety(TalonFXControlMode.PercentOutput, .15);
  }

  public void receiveFromSingleSubstation() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, 13740);
  }

  public void stop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
