// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm2 extends SubsystemBase {
  /** Creates a new Arm2. */
  private static final class Constants {
    private static final int armCanID = 12;
  }

  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);

  public Arm2() {
  }

  public void moveUp() {
    armMotor.set(TalonFXControlMode.PercentOutput, -.15);
  }

  public void moveDown() {
    armMotor.set(TalonFXControlMode.PercentOutput, .15);
  }

  public void stop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
