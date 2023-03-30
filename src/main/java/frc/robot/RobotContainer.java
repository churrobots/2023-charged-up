// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.YahtzeeBalance;
import frc.robot.helpers.GreenIsolator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
    createAutonomousSelector();
    GreenIsolator.createCameraStream("Sonic");
  }

  private void configureButtonBindings() {

    // Teleop commands for the driver and operator.

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

    double slowDriveScaling = 0.4;
    Command slowAndSteadyPeople = new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            true, true),
        m_drivetrain);

    // Driver
    var startButton = new JoystickButton(m_driverController, Button.kStart.value);
    var leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

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

    Command safelyRestTheArm = new RunCommand(m_arm::restTheArm, m_arm);
    Command stopRollers = new RunCommand(m_intake::stopThePlan, m_intake);

    // Set defaults for all subsystems
    m_drivetrain.setDefaultCommand(driveFieldRelativeWithJoysticks);
    m_arm.setDefaultCommand(safelyRestTheArm);
    m_intake.setDefaultCommand(stopRollers);
  }

  /**
   * Add the command corresponding to following the given trajectory.
   * If the path does not exist by that name, will add a no-op command
   * so that at least the robot does not crash on bootup.
   * 
   * @param name
   * @return Command
   */
  private void addAutoCommandToSelector(String name) {
    Command autoPathCommand;
    try {
      PathPlannerTrajectory trajectory = PathPlanner.loadPath(name,
          m_drivetrain.getPathPlannerConstraints());
      autoPathCommand = m_drivetrain.followTrajectoryCommand(trajectory, true);
    } catch (Error err) {
      // TODO: set some SmartDashboard state so the LightShow can detect an error
      var noopCommand = new InstantCommand();
      autoPathCommand = noopCommand;
    }
    m_autoPathChoice.addOption(name, autoPathCommand);
  }

  /**
   * Add the command corresponding to following the given trajectory.
   * If the path does not exist by that name, will add a no-op command
   * so that at least the robot does not crash on bootup.
   * 
   * This will honor Event Markers on the path using AutoBuilder
   * https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#autobuilder
   * 
   * @param name
   * @return Command
   */
  private void addAutoCommandToSelector(String name, SwerveAutoBuilder builder) {
    Command autoPathCommand;
    try {
      List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name,
          m_drivetrain.getPathPlannerConstraintsForAutoBuilder());
      autoPathCommand = builder.fullAuto(pathGroup);
    } catch (Error err) {
      // TODO: set some SmartDashboard state so the LightShow can detect an error
      var noopCommand = new InstantCommand();
      autoPathCommand = noopCommand;
    }
    m_autoPathChoice.addOption(name, autoPathCommand);
  }

  private void createAutonomousSelector() {

    // Create an AutoBuilder we can use to execute commands along a path.
    // https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#autobuilder
    // Global event map for all autobuilder commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    var autoPickup = new RunCommand(m_arm::receiveFromGround, m_arm)
        .alongWith(new RunCommand(m_intake::yoinkTheCubes, m_intake))
        .withTimeout(1.2)
        .andThen(new InstantCommand(m_intake::stopThePlan, m_intake))
        .andThen(new RunCommand(m_arm::restTheArm, m_arm));

    var autoYeetBottom = new RunCommand(m_arm::moveToLow, m_arm)
        .withTimeout(.5)
        .andThen(new RunCommand(m_intake::yeetTheCubes, m_intake).withTimeout(1))
        .andThen(new InstantCommand(m_intake::stopThePlan, m_intake))
        .andThen(new RunCommand(m_arm::restTheArm, m_arm));

    eventMap.put("AutoPick", autoPickup);
    eventMap.put("YeetBottom", autoYeetBottom);

    var kPX = 1.2;
    var kPTheta = 1.1;
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_drivetrain::getPose,
        m_drivetrain::resetPose,
        m_drivetrain.getKinematics(),
        new PIDConstants(kPX, 0.0, 0.0),
        new PIDConstants(kPTheta, 0.0, 0.0),
        m_drivetrain::setModuleStates,
        eventMap,
        true,
        m_drivetrain);

    // Add selector for choosing a trajectory to run.
    m_autoPathChoice.addOption("Do Nothing", new InstantCommand());

    addAutoCommandToSelector("BlueCenter&Leave");
    addAutoCommandToSelector("BlueNear&Leave");
    addAutoCommandToSelector("BlueFar&Leave");
    addAutoCommandToSelector("BlueCenter&Balance");

    addAutoCommandToSelector("RedCenter&Leave");
    addAutoCommandToSelector("RedNear&Leave");
    addAutoCommandToSelector("RedFar&Leave");
    addAutoCommandToSelector("RedCenter&Balance");

    // Paths involving events
    addAutoCommandToSelector("MULTI_PickAndBalance", autoBuilder);
    addAutoCommandToSelector("MULTI_NearSidePick", autoBuilder);
    addAutoCommandToSelector("MULTI_NearSide2Piece", autoBuilder);
    addAutoCommandToSelector("MULTI_BumpSidePick", autoBuilder);
    addAutoCommandToSelector("MULTI_BumpSide2Piece", autoBuilder); // needs tuning

    // These were experiments from drive practice night
    // addAutoCommandToSelector("TEST_CenterPickBalance", autoBuilder);
    // addAutoCommandToSelector("TEST_NearSide2Piece", autoBuilder);

    // Uncomment these if you want to run more calibrations
    // addAutoCommandToSelector("CALIBRATE_Linear", autoBuilder);
    // addAutoCommandToSelector("CALIBRATE_Rotate", autoBuilder);
    // addAutoCommandToSelector("CALIBRATE_LinearRotate", autoBuilder);
    // addAutoCommandToSelector("CALIBRATE_Figure8", autoBuilder);

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
    gotoScoringPosition = gotoScoringPosition.withTimeout(.50);

    // Prepare the rest of our actions.
    Command resetDrivetrainGyroSoFieldRelativeWorksAftward = new RunCommand(m_drivetrain::resetGyroForAuto,
        m_drivetrain).withTimeout(0.25);
    Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm).withTimeout(0.25);
    Command yeetForSomeTime = new RunCommand(m_intake::yeetTheCubes, m_intake).withTimeout(0.75);
    Command stopRollers = new InstantCommand(m_intake::stopThePlan, m_intake);
    Command resetArm = new RunCommand(m_arm::restTheArm, m_arm).withTimeout(0.50);
    Command stopTheArm = new InstantCommand(m_arm::restTheArm, m_arm);
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

}