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
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.Tunables;

public class Arm extends SubsystemBase {

  enum Level {
    LOW,
    MID,
    PARTY
  }

  private static final class Constants {
    private static final int armCanID = 12;
    private static final double calibrationVelocitySensorUnitsPerSecond = -1000;
    // private static final int midCounts = 8000;
    // private static final int lowCounts = 11975;
    // private static final int substationCounts = 10000;
    // private static final int partyCounts = 8750;
  }

  private final SubsystemInspector m_inspector = new SubsystemInspector(getSubsystem());
  private Level level = Level.LOW;

  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm() {
    armMotor.configFactoryDefault();
    var safeCurrentLimitsForFalcon = new StatorCurrentLimitConfiguration(true, 32, 35, 2.5);
    armMotor.configStatorCurrentLimit(safeCurrentLimitsForFalcon);
    armMotor.setNeutralMode(NeutralMode.Brake);
    updateArmTuning();
  }

  private void runMotorWithSafety(TalonFXControlMode mode, double value) {
    if (m_isCalibrated) {
      if (mode == TalonFXControlMode.MotionMagic) {
        if (value > Tunables.kMidCounts.get() - 1000 && value <= Tunables.kMidCounts.get() + 1500) {
          level = Level.MID;
        } else if (value > Tunables.kMidCounts.get() + 1500) {
          level = Level.LOW;
        } else {
          level = Level.PARTY;
        }
        int kMeasuredPosHorizontal = 22673; // Position measured when arm is horizontal
        double kTicksPerDegree = 53828 / 360; // Sensor is 1:1 with arm rotation
        double currentPos = armMotor.getSelectedSensorPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);
        double maxGravityFF = -0.07;
        armMotor.set(mode, value, DemandType.ArbitraryFeedForward,
            maxGravityFF * cosineScalar);
        m_inspector.set("target", value);
        m_inspector.set("actual", armMotor.getSelectedSensorPosition());
      } else {
        armMotor.set(mode, value);
      }
    }
  }

  public boolean isShootingMid() {
    return level == Level.MID;
  }

  public boolean isShootingGround() {
    return level == Level.LOW;
  }

  public boolean isPartying() {
    return level == Level.PARTY;
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
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kSubstationCounts.get());
  }

  public void receiveFromSingleSubstation(double offset) {
    offset *= 1000;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kSubstationCounts.get() + offset);
  }

  public void moveToLow() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kLowCounts.get());
  }

  public void moveToLow(double offset) {
    offset *= 1000;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kLowCounts.get() + offset);
  }

  public void moveToMid() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kMidCounts.get());

  }

  public void moveToMid(double offset) {
    offset *= 1000;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kMidCounts.get() + offset);
  }

  public void moveToParty() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kPartyCounts.get());
  }

  public void moveToParty(double offset) {
    offset *= 1000;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Tunables.kPartyCounts.get() + offset);
  }

  public void stop() {
    if (armMotor.getSelectedSensorPosition() > 6000) {
      runMotorWithSafety(TalonFXControlMode.MotionMagic, 3000);
    } else {
      armMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean didChange = Tunables.fastAndFaster.didChange() ||
        Tunables.monsterInject.didChange() ||
        Tunables.kP.didChange() ||
        Tunables.kF.didChange() ||
        Tunables.kI.didChange() ||
        Tunables.kD.didChange();

    if (didChange) {
      updateArmTuning();
    }
  }

  private void updateArmTuning() {
    FalconHelper.configureMotionMagic(
        armMotor,
        Tunables.fastAndFaster.get(),
        Tunables.monsterInject.get(),
        Tunables.kSmoothing.get(),
        Tunables.kP.get(),
        Tunables.kF.get(),
        Tunables.kI.get(),
        Tunables.kD.get());
  }

}
