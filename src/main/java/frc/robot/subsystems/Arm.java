// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.FalconHelper;
import frc.robot.helpers.Tunables;

public class Arm extends SubsystemBase {

  private static final class Constants {
    private static final int armCanID = 12;
    private static final double calibrationVelocitySensorUnitsPerSecond = -1000;
  }

  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm() {
    armMotor.configFactoryDefault();
    var safeCurrentLimitsForFalcon = new StatorCurrentLimitConfiguration(true, 40, 45, 2.5);
    armMotor.configStatorCurrentLimit(safeCurrentLimitsForFalcon);
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  private void runMotorWithSafety(TalonFXControlMode mode, double value) {
    if (m_isCalibrated) {
      if (mode == TalonFXControlMode.MotionMagic) {
        int kMeasuredPosHorizontal = 22673; // Position measured when arm is horizontal
        double kTicksPerDegree = 53828 / 360; // Sensor is 1:1 with arm rotation
        double currentPos = armMotor.getSelectedSensorPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);
        double maxGravityFF = 0.07;
        armMotor.set(mode, value, DemandType.ArbitraryFeedForward,
            maxGravityFF * cosineScalar);
      } else {
        armMotor.set(mode, value);
      }
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

  public void receiveFromSingleSubstation() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, 10000);
  }

  public void moveToLow() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, 11757);
  }

  public void moveToMid() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, 8500);

  }

  public void resetArm() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, 0);
  }

  public void stop() {
    if (armMotor.getSelectedSensorPosition() > 6000) {
      resetArm();
    } else {
      armMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    FalconHelper.configureMotionMagic(
        armMotor,
        Tunables.fastAndFaster.get(),
        Tunables.monsterInject.get(),
        1,
        Tunables.kP.get(),
        Tunables.kF.get(),
        Tunables.kI.get(),
        Tunables.kD.get());
  }

}
