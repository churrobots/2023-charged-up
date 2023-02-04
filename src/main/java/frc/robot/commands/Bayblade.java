package frc.robot.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Bayblade extends CommandBase {
  DriveSubsystem car;
  Double angle;
  XboxController controller;

  public Bayblade(DriveSubsystem car, XboxController controller) {
    this.car = car;
    this.controller = controller;
    this.addRequirements(car);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d angleDifference = getDesiredAngle().minus(getCurrentAngle());
    boolean counterclockwise = true;
    if (angleDifference.getDegrees() > 0) {
      counterclockwise = false;
    }
    car.spinInPlace(counterclockwise);
    // this.car.setModulePositions(new SwerveModulePosition[] {
    // new SwerveModulePosition(0, getAngle()),
    // new SwerveModulePosition(0, getAngle()),
    // new SwerveModulePosition(0, getAngle()),
    // new SwerveModulePosition(0, getAngle())
    // });
  }

  private Rotation2d getDesiredAngle() {
    double angle = Math.atan2(controller.getRightY(), controller.getRightX());
    return new Rotation2d(angle);

  }

  private Rotation2d getCurrentAngle() {
    return car.getGyroAngle();
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