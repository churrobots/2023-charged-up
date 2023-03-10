// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final class Constants {
    private static final int topRollerMotorID = 10;
    private static final int bottomRollerMotorID = 11;
  }

  private final WPI_VictorSPX topCubeYoinker = new WPI_VictorSPX(Constants.topRollerMotorID);
  private final WPI_VictorSPX bottomCubeYoinker = new WPI_VictorSPX(Constants.bottomRollerMotorID);

  public Intake() {
  }

  public void yoinkTheCubes() {
    topCubeYoinker.set(.75);
    bottomCubeYoinker.set(.75);
    
  }

  public void yeetTheCubes() {
    topCubeYoinker.set(-.75);
    bottomCubeYoinker.set(-.75);

  }

  public void stopThePlan() {
    topCubeYoinker.set(0);
    bottomCubeYoinker.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
