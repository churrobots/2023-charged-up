package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AngleTwister extends CommandBase {
  DriveSubsystem car;
  Double angle;

  public AngleTwister(DriveSubsystem car, Double angle) {
    this.car = car;
    this.angle = angle;
    this.addRequirements(car);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.car.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0.1, new Rotation2d(angle)),
        new SwerveModuleState(0.1, new Rotation2d(angle)),
        new SwerveModuleState(0.1, new Rotation2d(angle)),
        new SwerveModuleState(0.1, new Rotation2d(angle))
    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}