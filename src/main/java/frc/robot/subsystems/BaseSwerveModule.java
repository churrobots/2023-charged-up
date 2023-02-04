package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

abstract public class BaseSwerveModule {

  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract void setDesiredState(SwerveModuleState desiredState);

  public abstract void setDesiredPosition(SwerveModulePosition desiredPosition);

  public abstract void resetEncoders();

  public abstract Rotation2d getAngle();
}