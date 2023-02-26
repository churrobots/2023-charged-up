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
import frc.robot.helpers.SubsystemInspector;

public class Arm2 extends SubsystemBase {
  /** Creates a new Arm2. */
  private static final class Constants {
    private static final int armCanID = 12;
  }

  private final SubsystemInspector inspector = new SubsystemInspector(getSubsystem());
  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm2() {
    armMotor.configFactoryDefault();
    var safeCurrentLimitsForFalcon = new StatorCurrentLimitConfiguration(true, 35, 40, 2.5);
    armMotor.configStatorCurrentLimit(safeCurrentLimitsForFalcon);
    armMotor.setNeutralMode(NeutralMode.Brake);
    configureMotionMagic();
  }

  private void configureMotionMagic() {
    // This sets a lot of the defaults that the example code seems to require
    // for full functioning of the Falcon500s. Cargo culting FTW.
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    // TODO: calculate or characterize these values? why would you ever not use the
    // 0th slots?
    int fakeSlot = 0;
    int fakePIDSlot = 0;
    int fakeTimeoutMilliseconds = 30;
    double fakeKP = 0.2;
    double fakeKI = 0.0;
    double fakeKD = 0.0;
    double fakeKF = 0.2;

    armMotor.configNeutralDeadband(0.001); // really low deadzone

    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, fakeTimeoutMilliseconds);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, fakeTimeoutMilliseconds);

    armMotor.selectProfileSlot(fakeSlot, fakePIDSlot);
    armMotor.config_kF(fakeSlot, fakeKF, fakeTimeoutMilliseconds);
    armMotor.config_kP(fakeSlot, fakeKP, fakeTimeoutMilliseconds);
    armMotor.config_kI(fakeSlot, fakeKI, fakeTimeoutMilliseconds);
    armMotor.config_kD(fakeSlot, fakeKD, fakeTimeoutMilliseconds);

    armMotor.configMotionCruiseVelocity(18000, fakeTimeoutMilliseconds);
    armMotor.configMotionAcceleration(8000, fakeTimeoutMilliseconds);
    armMotor.configMotionSCurveStrength(3);

    armMotor.configPeakOutputForward(0.4);
    armMotor.configPeakOutputReverse(-0.4);

  }

  public void manuallyCalibrate() {
    inspector.set("activity", "manuallyCalibrate");
    armMotor.set(TalonFXControlMode.PercentOutput, -.15);
    armMotor.setSelectedSensorPosition(0);
    m_isCalibrated = true;
  }

  public void moveUp() {
    inspector.set("activity", "moveUp");
    armMotor.set(TalonFXControlMode.PercentOutput, -.15);
  }

  public void moveDown() {
    inspector.set("activity", "moveDown");
    armMotor.set(TalonFXControlMode.PercentOutput, .15);
  }

  public void receiveFromSingleSubstation() {
    inspector.set("activity", "receiveFromSingleSubstation");
    armMotor.set(TalonFXControlMode.MotionMagic, 10000);
  }

  public void stop() {
    inspector.set("activity", "stop");
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
