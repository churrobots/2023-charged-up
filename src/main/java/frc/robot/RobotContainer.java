// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AngleSnap;
import frc.robot.commands.JengaBalance;
import frc.robot.commands.YahtzeeBalance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake(m_arm);
  private final LightShow m_lightShow = new LightShow(m_intake);

  SendableChooser<Command> m_autoScoringChoice = new SendableChooser<Command>();
  SendableChooser<Command> m_autoPathChoice = new SendableChooser<Command>();

  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);

  private double signedSquare(double val) {
    return val < 0 ? val * val * -1 : val * val;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
    createAutonomousSelector();
  }

  private void configureButtonBindings() {

    // Teleop commands for the driver and operator.
    Command turnButtonY = new AngleSnap(0, m_drivetrain);
    Command turnButtonB = new AngleSnap(-90, m_drivetrain);
    Command turnButtonA = new AngleSnap(180, m_drivetrain);
    Command turnButtonX = new AngleSnap(90, m_drivetrain);
    // Command setBalance = new JengaBalance(m_drivetrain);

    Command anchorInPlace = new RunCommand(() -> m_drivetrain.setXFormation(), m_drivetrain);
    Command resetGyro = new RunCommand(() -> m_drivetrain.resetGyro(), m_drivetrain);

    Command yeet = new RunCommand(m_intake::yeetTheCubes, m_intake);
    Command yoink = new RunCommand(m_intake::yoinkTheCubes, m_intake);

    Command moveArmIntoCalibration = new RunCommand(m_arm::moveIntoCalibrationPosition, m_arm);
    Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm);
    Command moveToReceive = new RunCommand(() -> m_arm.receiveFromSingleSubstation(-m_operatorController.getLeftY()),
        m_arm);
    Command moveToLow = new RunCommand(() -> m_arm.moveToLow(-m_operatorController.getLeftY()), m_arm);
    Command moveToMid = new RunCommand(() -> m_arm.moveToMid(-m_operatorController.getLeftY()), m_arm);

    double slowDriveScaling = 0.6;
    Command slowAndSteadyPeople = new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(signedSquare(m_driverController.getLeftY() * slowDriveScaling),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(signedSquare(m_driverController.getLeftX() * slowDriveScaling),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(),
                OIConstants.kDriveDeadband),
            true, true),
        m_drivetrain);

    // Driver
    var startButton = new JoystickButton(m_driverController, Button.kStart.value);
    var backButton = new JoystickButton(m_driverController, Button.kBack.value);
    var leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
    var aButton = new JoystickButton(m_driverController, Button.kA.value);
    var bButton = new JoystickButton(m_driverController, Button.kB.value);
    var yButton = new JoystickButton(m_driverController, Button.kY.value);
    var xButton = new JoystickButton(m_driverController, Button.kX.value);

    aButton.whileTrue(turnButtonA);
    bButton.whileTrue(turnButtonB);
    yButton.whileTrue(turnButtonY);
    xButton.whileTrue(turnButtonX);
    // backButton.whileTrue(setBalance);

    leftBumper.whileTrue(anchorInPlace);
    rightBumper.whileTrue(slowAndSteadyPeople);
    startButton.whileTrue(resetGyro);

    // Operator
    var leftBumperOpButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    var rightBumperOpButton = new JoystickButton(m_operatorController, Button.kRightBumper.value);
    var aOpButton = new JoystickButton(m_operatorController, Button.kA.value);
    var xOpButton = new JoystickButton(m_operatorController, Button.kX.value);
    var yOpButton = new JoystickButton(m_operatorController, Button.kY.value);
    var startOpButton = new JoystickButton(m_operatorController, Button.kStart.value);
    var backOpButton = new JoystickButton(m_operatorController, Button.kBack.value);

    leftBumperOpButton.whileTrue(yoink);
    rightBumperOpButton.whileTrue(yeet);
    backOpButton.whileTrue(moveArmIntoCalibration);
    startOpButton.whileTrue(resetArmCalibration);
    xOpButton.whileTrue(moveToLow);
    aOpButton.whileTrue(moveToMid);
    yOpButton.whileTrue(moveToReceive);
  }

  private void ensureSubsystemsHaveDefaultCommands() {

    Command driveFieldRelativeWithJoysticks = new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(),
                OIConstants.kDriveDeadband),
            true, true),
        m_drivetrain);

    Command safelyRestTheArm = new RunCommand(m_arm::stop, m_arm);
    Command stopRollers = new RunCommand(m_intake::stopThePlan, m_intake);

    // Set defaults for all subsystems
    m_drivetrain.setDefaultCommand(driveFieldRelativeWithJoysticks);
    m_arm.setDefaultCommand(safelyRestTheArm);
    m_intake.setDefaultCommand(stopRollers);
  }

  private void addAutoCommandToSelector(String name) {
    Command autoPathCommand = safelyReadPathCommand(name);
    m_autoPathChoice.addOption(name, autoPathCommand);
  }

  private void createAutonomousSelector() {

    // Add selector for choosing a trajectory to run.
    addAutoCommandToSelector("BlueCenter&Leave");
    addAutoCommandToSelector("BlueNear&Leave");
    addAutoCommandToSelector("BlueFar&Leave");
    addAutoCommandToSelector("BlueCenter&Balance");
    addAutoCommandToSelector("RedCenter&Leave");
    addAutoCommandToSelector("RedNear&Leave");
    addAutoCommandToSelector("RedFar&Leave");
    addAutoCommandToSelector("RedCenter&Balance");
    Command dontMove = new InstantCommand();
    m_autoPathChoice.addOption("Do Nothing", dontMove);

    // Add selector for scoring low or mid.

    Command autoMoveToLow = new RunCommand(m_arm::moveToLow, m_arm);
    Command autoMoveToMid = new RunCommand(m_arm::moveToMid, m_arm);
    m_autoScoringChoice.addOption("ScoreLow", autoMoveToLow);
    m_autoScoringChoice.addOption("ScoreMid", autoMoveToMid);

    // Add these options to the interface
    SmartDashboard.putData(m_autoScoringChoice);
    SmartDashboard.putData(m_autoPathChoice);
  }

  /**
   * This is called by the system when automomous runs, and it
   * should return the command you want to execute when automous
   * mode begins.
   */
  public Command getAutonomousCommand() {

    // Get the selections from the drive team. Handle nulls safely.
    Command doThePath = m_autoPathChoice.getSelected();
    if (doThePath == null) {
      doThePath = new InstantCommand();
    }
    Command gotoScoringPosition = m_autoScoringChoice.getSelected();
    if (gotoScoringPosition == null) {
      gotoScoringPosition = new RunCommand(m_arm::moveToLow, m_arm);
    }
    gotoScoringPosition = gotoScoringPosition.withTimeout(2);

    // Prepare the rest of our actions.
    Command resetDrivetrainGyroSoFieldRelativeWorksAftward = new RunCommand(m_drivetrain::resetGyroForAuto,
        m_drivetrain).withTimeout(0.25);
    Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm).withTimeout(0.25);
    Command yeetForSomeTime = new RunCommand(m_intake::yeetTheCubes, m_intake).withTimeout(0.75);
    Command stopRollers = new InstantCommand(m_intake::stopThePlan, m_intake);
    Command resetArm = new RunCommand(m_arm::stop, m_arm).withTimeout(0.75);
    Command stopTheArm = new InstantCommand(m_arm::stop, m_arm);
    Command anchorJustInCaseWeAreBalancing = new RunCommand(m_drivetrain::setXFormation, m_drivetrain);
    Command tryToBalance = new YahtzeeBalance(m_drivetrain);

    // Sequence our actions so that we score first, and then perform our path.
    // At the end, we anchor so we don't slip off the charging station.
    return resetArmCalibration
        .andThen(resetDrivetrainGyroSoFieldRelativeWorksAftward)
        .andThen(gotoScoringPosition)
        .andThen(yeetForSomeTime)
        .andThen(stopRollers)
        .andThen(resetArm)
        .andThen(stopTheArm)
        .andThen(doThePath)
        .andThen(tryToBalance)
        .andThen(anchorJustInCaseWeAreBalancing);
  }

  /**
   * Get the command corresponding to following the given trajectory.
   * If the path does not exist by that name, will return a no-op command
   * so that at least the robot does not crash on bootup.
   * 
   * @param pathplannerName
   * @return Command
   */
  private Command safelyReadPathCommand(String pathplannerName) {
    try {
      PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathplannerName,
          m_drivetrain.getPathPlannerConstraints());
      return m_drivetrain.followTrajectoryCommand(trajectory, true);
    } catch (Error err) {
      // TODO: set some SmartDashboard state so the LightShow can detect an error
      var noopCommand = new InstantCommand();
      return noopCommand;
    }
  }

}