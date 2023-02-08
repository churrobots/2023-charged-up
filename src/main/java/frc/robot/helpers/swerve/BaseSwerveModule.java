package frc.robot.helpers.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract public class BaseSwerveModule extends SubsystemBase {
  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract void setDesiredState(SwerveModuleState desiredState);

  public abstract void resetDriveEncodersToZero();

  public abstract void assertModuleIsPointedForwardAndStoreCalibration();

}
