// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Gamepads;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriverPullupIntake;
import frc.robot.commands.OperatorManualLoad;
import frc.robot.commands.OperatorPullback;
import frc.robot.commands.OperatorPullupSensor;
import frc.robot.commands.OperatorRevThenPullback;
import frc.robot.commands.ShootSequenceFull;
import frc.robot.commands.TestLogCommand;
import frc.robot.commands.autoSequences.driveForward;
import frc.robot.commands.autoSequences.middleFourPieceBlue;
import frc.robot.commands.autoSequences.middleFourPieceRed;
import frc.robot.commands.autoSequences.middleThreePieceBlue;
import frc.robot.commands.autoSequences.middleThreePieceRed;
import frc.robot.commands.autoSequences.middleTwoPiece;
import frc.robot.commands.autoSequences.moveNoShoot;
import frc.robot.commands.autoSequences.shootMove;
import frc.robot.commands.autoSequences.sideLongAmpBlue;
import frc.robot.commands.autoSequences.sideLongAmpRed;
import frc.robot.commands.autoSequences.sideLongSourceBlue;
import frc.robot.commands.autoSequences.sideLongSourceRed;
import frc.robot.commands.autoSequences.sideThreePieceBlue;
import frc.robot.commands.autoSequences.sideThreePieceRed;
import frc.robot.commands.autoSequences.sideTwoPieceBlue;
import frc.robot.commands.autoSequences.sideTwoPieceRed;
import frc.robot.pathplanner.CommandChooser;
import frc.robot.subsystems.AnglerPIDSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Gamepads.DRIVER);
  private final CommandXboxController m_operatorController = new CommandXboxController(Gamepads.OPERATOR);

  public final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem(Constants.CAN.CLIMB);
  public final LedSubsystem m_led = new LedSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.CAN.LEFT_FLYWHEEL,
      Constants.CAN.RIGHT_FLYWHEEL);
  public final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.CAN.BELT, Constants.CAN.LEFT_INTAKE,
      Constants.CAN.RIGHT_INTAKE, m_led);

  public final AnglerPIDSubsystem m_angler = new AnglerPIDSubsystem();

  private double anglerIncrementValue = 0.25;

  SendableChooser<String> m_autoChooser = new SendableChooser<>();

  private String m_autoCommand = "";
  private Command m_autonomousCommand;

  private CommandChooser m_commandChooser = new CommandChooser();

  ShuffleboardTab dashboard = Shuffleboard.getTab("RobotContainer");
  // Configure the trigger bindings

  public GenericEntry waittime = dashboard.add("Time After Shoot to Wait (Seconds)", 0)
      .getEntry();

  public GenericEntry drivedistance = dashboard.add("Drivedistance (m)", 1)
      .getEntry();

  GenericEntry drivescale = dashboard.addPersistent("Drivescale", 0.3)
      .getEntry();

  GenericEntry driveMoreOffsetEntry = dashboard.addPersistent("Drive More Offset", 0.07)
      .getEntry();

  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser.setDefaultOption("Shoot Then Drive", "shootMove");
    m_autoChooser.addOption("Just Forwards", "driveForward");
    m_autoChooser.addOption("No Shoot Drive", "moveNoShoot");
    m_autoChooser.addOption("Middle Two Piece", "middleTwoPiece");
    m_autoChooser.addOption("Middle Three Piece Red", "middleThreePieceRed");
    m_autoChooser.addOption("Path Planner Example", "pathPlannerExample");
    m_autoChooser.addOption("Middle Three Piece Blue", "middleThreePieceBlue");
    m_autoChooser.addOption("Middle Four Piece Red", "middleFourPieceRed");
    m_autoChooser.addOption("Middle Four Piece Blue", "middleFourPieceBlue");
    m_autoChooser.addOption("Side Two Piece Red", "sideTwoPieceRed");
    m_autoChooser.addOption("Side Two Piece Blue", "sideTwoPieceBlue");
    m_autoChooser.addOption("Side Three Piece Red", "sideThreePieceRed");
    m_autoChooser.addOption("Side Three Piece Blue", "sideThreePieceBlue");
    m_autoChooser.addOption("Side Long Source Red", "sideLongSourceRed");
    m_autoChooser.addOption("Side Long Source Blue", "sideLongSourceBlue");
    m_autoChooser.addOption("Side Long Amp Blue", "sideLongAmpBlue");
    m_autoChooser.addOption("Side Long Amp Red", "sideLongAmpRed");
    m_autoChooser.addOption("Path Planner Straight", "pathPlannerStraight");
    m_autoChooser.addOption("Path Planner 4Piece", "pathPlanner4Piece");

    NamedCommands.registerCommand("shoot!", new ShootSequenceFull(m_shooter, m_intake, m_led));
    NamedCommands.registerCommand("testlog", new TestLogCommand());

    var commandChoosers = m_commandChooser.getChoosers();
    int rowIndex = 1;
    for (var entry : commandChoosers.entrySet()) {
      m_autoChooser.addOption(entry.getKey(), entry.getKey());
      dashboard.add(entry.getKey(), entry.getValue()).withSize(2, 1).withPosition(8, rowIndex);
      rowIndex = rowIndex + 1;
    }
    dashboard.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(8, 0);

    m_drivebase
        .setDefaultCommand(new DriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  // private double RightAxis(){
  // return m_driverController.getRightX();
  // }
  private void configureBindings() {

    // Driver Controller buttons
    // m_driverController.povUp().whileTrue(new RepeatCommand(new InstantCommand(()
    // -> m_climber.setClimber(0.5))));
    // m_driverController.povUp().whileFalse(new InstantCommand(() ->
    // m_climber.setClimber(0)));

    // m_driverController.povDown().whileTrue(new RepeatCommand(new
    // InstantCommand(() -> m_climber.setClimber(-0.5))));
    // m_driverController.povDown().whileFalse(new InstantCommand(() ->
    // m_climber.setClimber(0)));

    // m_driverController.a().onTrue(ShootSequenceWebAdj.create(m_shooter, m_intake,
    // m_led));

    // m_driverController.y().onTrue(ShootSequenceFull.create(m_shooter, m_intake,
    // m_led));

    // m_driverController.rightBumper().onTrue(ShootSequenceFullNoFlywheels.create(m_shooter,
    // m_intake, m_led));

    m_driverController.a().onTrue(ShootSequenceFull.create(m_shooter, m_intake, m_led));

    m_driverController.b().onTrue(new DriverPullupIntake(m_shooter, m_intake, m_led)
        .andThen(new OperatorRevThenPullback(m_shooter, m_intake, m_led)));

    // m_driverController.x().onTrue(new LoadShooterSequence(m_shooter, m_intake,
    // m_led));

    /*
     * m_driverController.b().whileTrue(
     * new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(0.7))
     * .andThen(new InstantCommand(() -> m_intake.setIntake(0.2)))));
     * 
     * m_driverController.b().onFalse(
     * new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(()
     * -> m_intake.setIntake(0))));
     */

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivebase.setScale(drivescale.getDouble(0.3))));
    m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_drivebase.setScale(1)));

    m_driverController.leftTrigger()
        .whileTrue(new RepeatCommand(new InstantCommand(() -> m_drivebase.set(0.175, m_driverController.getRightX()))));
    m_driverController.leftTrigger().onFalse(new InstantCommand(() -> m_drivebase.set(0, 0)));

    m_driverController.start().whileTrue(new RepeatCommand(new InstantCommand(() -> m_shooter.setFlywheels(-1))));
    m_driverController.start().onFalse(new InstantCommand(() -> m_shooter.setFlywheels(0)));

    // m_driverController.y().onTrue(new InstantCommand(() ->
    // m_shooter.setFlywheels(1)));
    // m_driverController.y().onFalse(new InstantCommand(() ->
    // m_shooter.setFlywheels(0)));

    // m_driverController.y().whileTrue(
    // new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(0.7))
    // .andThen(new InstantCommand(() -> m_intake.setIntake(0.2)))));
    // m_driverController.y().onFalse(
    // new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(()
    // -> m_intake.setIntake(0))));

    m_driverController.rightTrigger().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(-0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(-0.2)))));
    m_driverController.rightTrigger().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    // Operator Controller buttons
    // example led
    // m_operatorController.y().onTrue(new InstantCommand(() ->
    // m_led.setMode(Constants.LED.modes.badApple)));;

    // m_operatorController.b().onTrue(new InstantCommand(() ->
    // m_shooter.setFlywheels(-1)));
    // m_operatorController.b().onFalse(new InstantCommand(() ->
    // m_shooter.setFlywheels(0)));

    m_operatorController.b()
        .onTrue(new InstantCommand(() -> m_angler.angleSetpointWidget.setDouble(Constants.Angler.ZERO_SETPOINT)));

    m_operatorController.y().onTrue(
        new InstantCommand(() -> m_intake.stagedFlag = true).andThen(new OperatorPullback(m_shooter, m_intake, m_led)));

    m_operatorController.a().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(-0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(-0.2)))));
    m_operatorController.a().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    m_operatorController.x().onTrue(new OperatorManualLoad(m_shooter, m_intake, m_led));
    m_operatorController.x().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    // m_operatorController.leftStick().onTrue(new InstantCommand(() ->
    // m_led.setMode(Constants.LED.modes.whiteDotLines)));
    // m_operatorController.leftStick().onFalse(new InstantCommand(() ->
    // m_led.setMode(Constants.LED.modes.Rainbow)));

    m_operatorController.leftStick().onTrue(
        new InstantCommand(() -> m_angler.angleSetpointWidget.setDouble(Constants.Angler.LOW_HALF_CYCLE_SETPOINT)));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_shooter.setFlywheels(1)));

    m_operatorController.rightBumper().onTrue(new OperatorPullupSensor(m_shooter, m_intake, m_led)
        .andThen(new OperatorRevThenPullback(m_shooter, m_intake, m_led)));

    m_operatorController.rightStick().onTrue(new InstantCommand(() -> m_angler.resetAngleEncoder()));

    m_operatorController.start().onTrue(new InstantCommand(() -> m_shooter.instantStopFlywheels()));

    m_operatorController.back().onTrue(new InstantCommand(() -> anglerIncrementValue = 0.5));
    m_operatorController.back().onFalse(new InstantCommand(() -> anglerIncrementValue = 0.25));

    m_operatorController.leftTrigger().onTrue(new InstantCommand(() -> m_angler.angleSetpointWidget
        .setDouble(m_angler.angleSetpointWidget.getDouble(0) - anglerIncrementValue)));
    m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> m_angler.angleSetpointWidget
        .setDouble(m_angler.angleSetpointWidget.getDouble(0) + anglerIncrementValue)));

    m_operatorController.povLeft()
        .onTrue(new InstantCommand(() -> m_angler.angleSetpointWidget.setDouble(Constants.Angler.HALF_CYCLE_SETPOINT)));
    m_operatorController.povRight()
        .onTrue(new InstantCommand(() -> m_angler.angleSetpointWidget.setDouble(Constants.Angler.SPEAKER_SETPOINT)));

    m_operatorController.povUp().onTrue(new InstantCommand(() -> m_climber.setClimber(0.7)));
    m_operatorController.povUp().onFalse(new InstantCommand(() -> m_climber.setClimber(0)));

    m_operatorController.povDown().onTrue(new InstantCommand(() -> m_climber.setClimber(-1)));
    m_operatorController.povDown().onFalse(new InstantCommand(() -> m_climber.setClimber(0)));
  }

  public void onDisable() {
    m_drivebase.disable();
  }

  public void onTeleopInit() {
    m_drivebase.setDrivebaseIdle(IdleMode.kBrake);
    m_led.setMode(Constants.LED.modes.Rainbow);
    m_intake.setBelt(0);
    m_intake.setIntake(0);
    m_shooter.setFlywheels(0);
  }

  public void onSimulationInit() {
    m_drivebase.onSimulationInit();
  }

  // public void noteStage(){
  // new OperatorPullupSensor(m_shooter, m_intake, m_led)
  // .withTimeout(1.5).andThen(new OperatorPullback(m_shooter, m_intake, m_led))
  // .schedule();
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // If we can guarantee that we only change commands during disabled then we don't need
    // to potentially run updateAuto here and can just return the stored autonomous command.
    String autoCommand = m_autoChooser.getSelected();
    if (!m_autoCommand.equals(autoCommand) || m_commandChooser.hasCommandChanged(autoCommand)) {
      this.updateAuto();
    }
    return m_autonomousCommand;
  }

  void updateAuto() {
    String autoCommand = m_autoChooser.getSelected();
    if (!m_autoCommand.equals(autoCommand) || m_commandChooser.hasCommandChanged(autoCommand)) {
      m_autoCommand = autoCommand;
      // The selected command will be run in autonomous
      m_autonomousCommand = switch (autoCommand) {
        case "driveForward" -> new driveForward(m_drivebase);
        case "shootMove" ->
          new shootMove(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0), drivedistance.getDouble(1));
        case "moveNoShoot" ->
          new moveNoShoot(m_drivebase, m_shooter, m_led, waittime.getDouble(0), drivedistance.getDouble(1));
        case "middleTwoPiece" ->
          new middleTwoPiece(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0),
              drivedistance.getDouble(1));
        case "middleThreePieceRed" -> new middleThreePieceRed(m_drivebase, m_shooter, m_intake, m_led,
            waittime.getDouble(0), drivedistance.getDouble(1));
        case "middleThreePieceBlue" -> new middleThreePieceBlue(m_drivebase, m_shooter, m_intake, m_led,
            waittime.getDouble(0), drivedistance.getDouble(1));
        case "middleFourPieceRed" ->
          new middleFourPieceRed(m_drivebase, m_shooter, m_intake, m_led, driveMoreOffsetEntry.getDouble(0.07));
        case "middleFourPieceBlue" ->
          new middleFourPieceBlue(m_drivebase, m_shooter, m_intake, m_led, driveMoreOffsetEntry.getDouble(0.07));
        case "sideTwoPieceRed" -> new sideTwoPieceRed(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideTwoPieceBlue" -> new sideTwoPieceBlue(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideThreePieceRed" ->
          new sideThreePieceRed(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideThreePieceBlue" ->
          new sideThreePieceBlue(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideLongSourceRed" ->
          new sideLongSourceRed(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideLongSourceBlue" ->
          new sideLongSourceBlue(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideLongAmpBlue" -> new sideLongAmpBlue(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "sideLongAmpRed" -> new sideLongAmpRed(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0));
        case "pathPlannerExample" -> new PathPlannerAuto("New Auto");
        case "pathPlannerStraight" -> new PathPlannerAuto("New New Auto");
        case "pathPlanner4Piece" -> new PathPlannerAuto("testSeq");
        default -> m_commandChooser.setupAuto(autoCommand);
      };
    }
  }
}
