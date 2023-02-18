// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.Tunables;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RollBoth extends CommandBase {
  Arm arm;
  Intake intakeTop;
  Intake intakeBottom;

  /** Creates a new Score. */
  public RollBoth(Arm arm, Intake intakeTop, Intake intakeBottom) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intakeTop = intakeTop;
    this.intakeBottom = intakeBottom;
    addRequirements(arm);
    addRequirements(intakeTop);
    addRequirements(intakeBottom);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveToPositionWithMotionMagic(Tunables.armScorePositionSensorCounts.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isDoneWithMotionMagic()) {
      intakeTop.ejection();
      intakeBottom.ejection();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeTop.stopRollers();
    intakeBottom.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
