// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AngleSnap;
import frc.robot.commands.JengaBalance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;
import frc.robot.subsystems.DriveSubsystem.WhichDrivebase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final LightShow m_lightShow = new LightShow();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(WhichDrivebase.SpeedyHedgehog);
  private final Arm armTheSecond = new Arm();
  private final Intake intakeTheSecond = new Intake();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);
  // Create the autonomous chooser.
  SendableChooser<Command> scoringChoice = new SendableChooser<Command>();
  SendableChooser<Command> pathChoice = new SendableChooser<Command>();

  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  private static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  private double signedSquare(double val) {
    return val < 0 ? val * val * -1 : val * val;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure drivetrain
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(signedSquare(m_driverController.getLeftY()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(signedSquare(m_driverController.getLeftX()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(),
                    OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    var stopArm = new RunCommand(armTheSecond::stop, armTheSecond);
    armTheSecond.setDefaultCommand(stopArm);

    var stopRollers = new RunCommand(intakeTheSecond::stopThePlan, intakeTheSecond);
    intakeTheSecond.setDefaultCommand(stopRollers);

    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(armTheSecond);
    SmartDashboard.putData(intakeTheSecond);
    // creates Trajectory
    FollowPathWithEvents centerBalance = getPathCommand("Center&Balance", getScoreCommand());
    FollowPathWithEvents centerLeave = getPathCommand("Center&Leave", getScoreCommand());
    FollowPathWithEvents centerPrep = getPathCommand("Center&Prep", getScoreCommand());
    FollowPathWithEvents nearBalance = getPathCommand("Near&Balance", getScoreCommand());
    FollowPathWithEvents nearLeave = getPathCommand("Near&Leave", getScoreCommand());
    FollowPathWithEvents nearPrep = getPathCommand("Near&Prep", getScoreCommand());
    FollowPathWithEvents farBalance = getPathCommand("Far&Balance", getScoreCommand());
    FollowPathWithEvents farLeave = getPathCommand("Far&Leave", getScoreCommand());
    FollowPathWithEvents farPrep = getPathCommand("Far&Prep", getScoreCommand());
    FollowPathWithEvents garage = getPathCommand("TheMattGarageSpecial", getScoreCommand());

    Command moveToLow = new RunCommand(armTheSecond::moveToLow, armTheSecond);
    Command moveToMid = new RunCommand(armTheSecond::moveToMid, armTheSecond);

    // Sets up Auto
    pathChoice.addOption("Center&Balance", centerBalance);
    pathChoice.addOption("Center&Leave", centerLeave);
    pathChoice.addOption("Center&Prep", centerPrep);
    pathChoice.addOption("Near&Balance", nearBalance);
    pathChoice.addOption("Near&Leave", nearLeave);
    pathChoice.addOption("Near&Prep", nearPrep);
    pathChoice.addOption("Far&Balance", farBalance);
    pathChoice.addOption("Far&Leave", farLeave);
    pathChoice.addOption("Far&Prep", farPrep);
    pathChoice.addOption("TheGarageSpecial", garage);

    scoringChoice.addOption("ScoreLow", moveToLow);
    scoringChoice.addOption("ScoreMid", moveToMid);

    SmartDashboard.putData(scoringChoice);
    SmartDashboard.putData(pathChoice);

    var noLight = new InstantCommand(m_lightShow::runDefaultLights, m_lightShow);
    m_lightShow.setDefaultCommand(noLight);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    Command turnButtonY = new AngleSnap(0, m_robotDrive);
    Command turnButtonB = new AngleSnap(-90, m_robotDrive);
    Command turnButtonA = new AngleSnap(180, m_robotDrive);
    Command turnButtonX = new AngleSnap(90, m_robotDrive);
    Command setBalance = new JengaBalance(m_robotDrive, m_robotDrive.m_gyro);

    Command anchorInPlace = new RunCommand(() -> m_robotDrive.setX(), m_robotDrive);
    Command resetGyro = new RunCommand(() -> m_robotDrive.resetGyro(), m_robotDrive);

    Command yeet = new RunCommand(intakeTheSecond::yeetTheCubes, intakeTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(5, 0, 0), m_lightShow));
    Command yoink = new RunCommand(intakeTheSecond::yoinkTheCubes, intakeTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(0, 5, 0), m_lightShow));

    Command moveArmIntoCalibration = new RunCommand(armTheSecond::moveIntoCalibrationPosition, armTheSecond);
    Command resetArmCalibration = new RunCommand(armTheSecond::resetCalibration, armTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(10, 10, 10), m_lightShow));
    Command pickHigh = new RunCommand(armTheSecond::receiveFromSingleSubstation, armTheSecond);
    Command moveToLow = new RunCommand(armTheSecond::moveToLow, armTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(0, 0, 5), m_lightShow));
    Command moveToMid = new RunCommand(armTheSecond::moveToMid, armTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(0, 5, 5), m_lightShow));

    var startButton = new JoystickButton(m_driverController, Button.kStart.value);
    var backButton = new JoystickButton(m_driverController, Button.kBack.value);
    var leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

    // Driver
    var aButton = new JoystickButton(m_driverController, Button.kA.value);
    var bButton = new JoystickButton(m_driverController, Button.kB.value);
    var yButton = new JoystickButton(m_driverController, Button.kY.value);
    var xButton = new JoystickButton(m_driverController, Button.kX.value);

    aButton.whileTrue(turnButtonA);
    bButton.whileTrue(turnButtonB);
    yButton.whileTrue(turnButtonY);
    xButton.whileTrue(turnButtonX);
    backButton.whileTrue(setBalance);

    leftBumper.whileTrue(anchorInPlace);
    rightBumper.whileTrue(anchorInPlace);
    startButton.whileTrue(resetGyro);

    // Operator
    var leftBumperOpButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    var rightBumperOpButton = new JoystickButton(m_operatorController, Button.kRightBumper.value);
    var aOpButton = new JoystickButton(m_operatorController, Button.kA.value);
    var xOpButton = new JoystickButton(m_operatorController, Button.kX.value);
    var bOpButton = new JoystickButton(m_operatorController, Button.kB.value);
    var yOpButton = new JoystickButton(m_operatorController, Button.kY.value);
    var startOpButton = new JoystickButton(m_operatorController, Button.kStart.value);
    var backOpButton = new JoystickButton(m_operatorController, Button.kBack.value);

    leftBumperOpButton.whileTrue(yoink);
    rightBumperOpButton.whileTrue(yeet);
    backOpButton.whileTrue(moveArmIntoCalibration);
    startOpButton.whileTrue(resetArmCalibration);
    xOpButton.whileTrue(moveToMid);
    bOpButton.whileTrue(moveToLow);
    yOpButton.whileTrue(pickHigh);

    // TODO: give a "slow precise" mode for driver
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathChoice.getSelected();
  }

  public Command getScoreCommand() {
    return scoringChoice.getSelected();
  }

  public FollowPathWithEvents getPathCommand(String name, Command moveArm) {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(name,
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    HashMap<String, Command> eventMap = new HashMap<>();

    Command score = new RunCommand(intakeTheSecond::yeetTheCubes, intakeTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(0, 0, 5), m_lightShow));
    Command resetArm = new RunCommand(armTheSecond::resetArm, armTheSecond)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(10, 0, 5), m_lightShow));
    Command setBalance = new JengaBalance(m_robotDrive, m_robotDrive.m_gyro)
        .alongWith(new RunCommand(() -> m_lightShow.fillPercentage(0, 10, 5), m_lightShow));

    eventMap.put("MoveArm", moveArm);
    eventMap.put("Score", score);
    eventMap.put("ResetArm", resetArm);
    eventMap.put("Balance", setBalance);

    FollowPathWithEvents command = new FollowPathWithEvents(
        getPathFollowingCommand(examplePath),
        examplePath.getMarkers(),
        eventMap);
    return command;
  }

  public Command getPathFollowingCommand(PathPlannerTrajectory traj) {
    return m_robotDrive.followTrajectoryCommand(traj, true);
  }

}