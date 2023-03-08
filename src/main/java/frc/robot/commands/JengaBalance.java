package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class JengaBalance extends PIDCommand {
  Drivetrain car;
  WPI_Pigeon2 pigeon;
  Double angle;
  private final static double kDrivingP = 0.04;
  private final static double kDrivingI = 0;
  private final static double kDrivingD = 0;
  public final static double kTurnToleranceDeg = 5;
  public final static double kTurnRateToleranceDegPerS = 10;

  public JengaBalance(Drivetrain drive) {
    super(
        new PIDController(kDrivingP, kDrivingI, kDrivingD),
        // Read gyro pitch
        drive::getPitch,
        // Setpoint when gyro is level
        0,
        // Pipe output to turn robot
        output -> drive.drive(output, 0, 0, true, true),
        // Require the drive
        drive);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();

  }

}
