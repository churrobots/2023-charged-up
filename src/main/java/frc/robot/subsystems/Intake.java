// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Tunables;
import frc.robot.helpers.Tuner.TunableDouble;

public class Intake extends SubsystemBase {
  public static final double collectionSpeedPercentage = 0.75;

  private final WPI_VictorSPX rollerMotor;

  /** Creates a new Intake. */
  public Intake(int canId, boolean isInverted) {
    rollerMotor = new WPI_VictorSPX(canId);
    rollerMotor.setInverted(isInverted);
  }

  public void collectballs() {
    this.rollerMotor.set(collectionSpeedPercentage);

  }

  public void ejection() {
    this.rollerMotor.set(Tunables.ejectionSpeedPercentage.get());
  }

  public void stopRollers() {
    this.rollerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotState.isDisabled()) {
      stopRollers();
    }
  }
}
