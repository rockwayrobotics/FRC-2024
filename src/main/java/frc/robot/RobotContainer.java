// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.Shooter.ScoringMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.commands.autoSequences.*;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

enum AutoOption {
  driveForward,
  shootMove,
}

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

  private final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem(Constants.CAN.CLIMB);
  private final LedSubsystem m_led = new LedSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.CAN.GEAR, Constants.CAN.LEFT_FLYWHEEL, Constants.CAN.RIGHT_FLYWHEEL);
  private final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.CAN.BELT, Constants.CAN.LEFT_INTAKE, Constants.CAN.RIGHT_INTAKE);

  SendableChooser<AutoOption> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
    // Configure the trigger bindings

    m_autoChooser.setDefaultOption("Just Forwards", AutoOption.driveForward);
    m_autoChooser.addOption("Shoot Then Forwards", AutoOption.shootMove);
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
  private void configureBindings() {
    // Driver Controller buttons
    m_driverController.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(0.2))));
    m_driverController.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(-0.2))));
    m_driverController.povUp().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));
    m_driverController.povDown().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));

    m_driverController.leftBumper().onTrue(
      new InstantCommand(() -> m_drivebase.setScale(0.5))
      .andThen(new InstantCommand(() -> m_shooter.setFlywheelsScale(0.5))
      ));

    m_driverController.rightBumper().onTrue(new ShootSequence(m_shooter, m_intake)); 

    m_driverController.b().onTrue(new LoadShooterSequence(m_shooter, m_intake));

    m_driverController.a().whileTrue(
      new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(0.7))
      .andThen(new InstantCommand(() -> m_intake.setIntake(0.2)))
      ));

    m_driverController.x().whileTrue(
      new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(-0.7))
      .andThen(new InstantCommand(() -> m_intake.setIntake(-0.2)))
      ));

    m_driverController.y().whileTrue(new RepeatCommand(new InstantCommand(() -> m_shooter.setFlywheels(1))));


    // m_driverController.a().whileFalse(new InstantCommand(() -> m_shooter.setFlywheels(0)));

    m_driverController.leftBumper().onFalse(
      new InstantCommand(() -> m_drivebase.setScale(1))
      .andThen(new InstantCommand(() -> m_shooter.setFlywheelsScale(1))
      ));

    m_driverController.b().onFalse(new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));
    m_driverController.x().onFalse(new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));
    m_driverController.y().onFalse(new InstantCommand(() -> m_shooter.setFlywheels(0)));
    
    //m_driverController.rightBumper().whileFalse(new LoadShooterSequence(m_shooter, m_intake));
    //m_driverController.x().whileFalse(new InstantCommand(() -> m_intake.setIntake(0)));

    // Operator Controller buttons
    m_operatorController.a().onTrue(new InstantCommand(() -> m_shooter.setScoringMode(ScoringMode.SPEAKER)).andThen(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingYellow))));
    m_operatorController.b().onTrue(new InstantCommand(() -> m_shooter.setScoringMode(ScoringMode.AMP)).andThen(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow))));
    m_operatorController.x().onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingYellow)));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.badApple)));

    m_operatorController.leftBumper().whileTrue(new ShooterAngle(m_shooter, 0.2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return switch (m_autoChooser.getSelected()) {
      case driveForward -> new driveForward(m_drivebase);
      case shootMove -> new driveForward(m_drivebase);
    };
  }
}
