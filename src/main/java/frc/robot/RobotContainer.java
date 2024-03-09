// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.Shooter.ScoringMode;
import edu.wpi.first.networktables.GenericEntry;
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
  moveNoShoot,
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
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.CAN.GEAR, Constants.CAN.LEFT_FLYWHEEL,
      Constants.CAN.RIGHT_FLYWHEEL);
  private final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.CAN.BELT, Constants.CAN.LEFT_INTAKE,
      Constants.CAN.RIGHT_INTAKE);

  SendableChooser<AutoOption> m_autoChooser = new SendableChooser<>();
    
  ShuffleboardTab dashboard = Shuffleboard.getTab("NewDashboard");
    // Configure the trigger bindings

    GenericEntry waittime =
      dashboard.add("Time After Shoot to Wait (Seconds)", 0)
         .getEntry();

    GenericEntry drivedistance=
      dashboard.add("Drivedistance (m)", 1)
         .getEntry();

    GenericEntry ampshotspeed=
      dashboard.add("Amp Shot Speed (P)", 0.18)
         .getEntry();
        
  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    m_autoChooser.setDefaultOption("Shoot Then Drive", AutoOption.shootMove);
    m_autoChooser.addOption("Just Forwards", AutoOption.driveForward);
    m_autoChooser.addOption("No Shoot Drive", AutoOption.moveNoShoot);
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
  //   return m_driverController.getRightX();
  // }

  private void configureBindings() {
    // Driver Controller buttons
    //m_driverController.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(0.5))));
    //m_driverController.povUp().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));

    //m_driverController.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(-0.5))));
    //m_driverController.povDown().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));

    m_driverController.a().onTrue(new ShootSequenceHalf(m_shooter, m_intake, m_led,ampshotspeed.getDouble(0.18)));

    m_driverController.y().onTrue(new ShootSequenceFull(m_shooter, m_intake, m_led));

    //m_driverController.x().onTrue(new LoadShooterSequence(m_shooter, m_intake, m_led));

    m_driverController.b().onTrue(new LoadShooterSequenceNoReverse(m_shooter, m_intake, m_led));

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivebase.setScale(0.20)));
    m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_drivebase.setScale(1)));

    m_driverController.leftTrigger().whileTrue(new RepeatCommand(new InstantCommand(() -> m_drivebase.set(0.125, m_driverController.getRightX()))));
    m_driverController.leftTrigger().onFalse(new InstantCommand(() -> m_drivebase.set(0,0)));

    m_driverController.start().whileTrue(new RepeatCommand(new InstantCommand(() -> m_shooter.setFlywheels(-1))));
    m_driverController.start().onFalse(new InstantCommand(() -> m_shooter.setFlywheels(0)));

    m_driverController.rightBumper().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(0.2)))));
    m_driverController.rightBumper().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    m_driverController.rightTrigger().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(-0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(-0.2)))));
    m_driverController.rightTrigger().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    // Operator Controller buttons
    // example led
    // m_operatorController.y().onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.badApple)));;

    m_operatorController.b().onTrue(new LoadShooterSequenceNoReverse(m_shooter, m_intake, m_led));

    m_operatorController.y().whileTrue(new RepeatCommand(new InstantCommand(() -> m_shooter.setFlywheels(-1))));
    m_operatorController.y().onFalse(new InstantCommand(() -> m_shooter.setFlywheels(0)));

    m_operatorController.a().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(-0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(-0.2)))));
    m_operatorController.a().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));

    m_operatorController.x().whileTrue(
        new RepeatCommand(new InstantCommand(() -> m_intake.setBelt(0.7))
            .andThen(new InstantCommand(() -> m_intake.setIntake(0.2)))));
    m_operatorController.x().onFalse(
        new InstantCommand(() -> m_intake.setBelt(0)).andThen(new InstantCommand(() -> m_intake.setIntake(0))));


    m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.whiteDotLines)));
    m_operatorController.rightBumper().onFalse(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Off)));
    m_operatorController.leftBumper().onFalse(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));


    // TODO: angle STUFF HERE :3 

    m_operatorController.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(0.5))));
    m_operatorController.povUp().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));

    m_operatorController.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> m_climber.setClimber(-0.5))));
    m_operatorController.povDown().whileFalse(new InstantCommand(() -> m_climber.setClimber(0)));

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
      case shootMove -> new shootMove(m_drivebase, m_shooter, m_intake, m_led, waittime.getDouble(0), drivedistance.getDouble(1));
      case moveNoShoot -> new moveNoShoot(m_drivebase, m_shooter, m_led, waittime.getDouble(0), drivedistance.getDouble(1));
    };
  }
}
