// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class YahtzeeBalance extends CommandBase {

  private final Drivetrain m_drivetrain;

  public YahtzeeBalance(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: check advice on this thread, some said PID (Jenga) doesn't work
    // https://www.chiefdelphi.com/t/what-is-your-approach-to-automate-charging-station-balancing/427432/22
    // I have it drive onto the charge station fairly fast, and once the angle
    // exceeds about 13 degrees or so (it was higher but the robot once flew off the
    // other side of the charge station) it slows down to around 0.1 meters per
    // second and once the gyro reading is within 12 degrees or so of zero, it
    // stops, locks the wheels, and the charge station levels out. You’d have to
    // tune it for your robot since ours is about 124 pounds.
    // From my experience, the charge station reacts too slowly to make PIDs useful
    // since the charge station won’t balance out until the robot’s already driven
    // too far. At first I thought this solution would be temporary since I bodged
    // it together in a few hours’ time, but at our competition we probably had the
    // most efficient auto balance.
    // https://www.chiefdelphi.com/t/what-is-your-approach-to-automate-charging-station-balancing/427432/24
    // We did a similar thing that @LoganKippnick did. Ours differs in that it
    // initially goes roughly 1.4 ft/sec and it waits until we passed are over 11.5
    // degrees then it just switches to a P controller on gyro pitch on the roborio.
    // What’s crucial is that we clamp the output to never accelerate more than 0.89
    // ft/sec. It is always running the gyro pid then until the end of autonomous.
    var angle = m_drivetrain.getPitch();
    if (angle < -6.00) {
      m_drivetrain.drive(0.05, 0.0, 0, false, true);
    } else if (angle > 6.00) {
      m_drivetrain.drive(-0.05, 0.0, 0, false, true);
    } else {
      m_drivetrain.setXFormation();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: make sure to stop the drivetrain at this point
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
