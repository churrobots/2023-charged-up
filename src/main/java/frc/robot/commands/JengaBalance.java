package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class JengaBalance extends PIDCommand {

  private final static double kDrivingP = 0.04;
  private final static double kDrivingI = 0;
  private final static double kDrivingD = 0;

  public JengaBalance(Drivetrain drive) {
    super(
        new PIDController(kDrivingP, kDrivingI, kDrivingD),
        // Read gyro pitch
        drive::getPitch,
        // Setpoint when gyro is level
        0,
        // Pipe output to turn robot
        output -> drive.drive(output, 0, 0, false, true),
        // Require the drive
        drive);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

}
