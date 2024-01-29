// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

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
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MotorSubsystem m_MotorSubsystem = new MotorSubsystem(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Gamepads.DRIVER);
  private final CommandXboxController m_operatorController = new CommandXboxController(Gamepads.OPERATOR);

  private final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();
  
  ShuffleboardTab motors = Shuffleboard.getTab("Motors");

    GenericEntry talonSpeed =
      motors.add("Talon Speed", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    GenericEntry sparkSpeed =
      motors.add("Spark Speed", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Drive Controller buttons
    m_driverController.a().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.x().whileTrue(new RepeatCommand(new InstantCommand(() -> m_MotorSubsystem.setTalonSpeed(talonSpeed.getDouble(0)))));
    m_driverController.y().whileTrue(new RepeatCommand(new InstantCommand(() -> m_MotorSubsystem.setSparkSpeed(sparkSpeed.getDouble(0)))));
    m_driverController.x().whileFalse(new InstantCommand(() -> m_MotorSubsystem.setTalonSpeed(0)));
    m_driverController.y().whileFalse(new InstantCommand(() -> m_MotorSubsystem.setSparkSpeed(0)));


    // Operator Controller buttons
    m_operatorController.a().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
