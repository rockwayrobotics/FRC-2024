// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.subsystems.*;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MotorSubsystem m_motors = new MotorSubsystem(); 
  // private final LimitswitchSubsystem m_LimitswitchSubsystem = new LimitswitchSubsystem(); 
  // private final ColourSensorSubsystem m_ColourSensorSubsystem = new ColourSensorSubsystem();

  // private final LedSubsystem m_LedSubsystem = new LedSubsystem();  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID m_dCtrl = new CommandGenericHID(Gamepads.DRIVER);
  // private final CommandXboxController m_operatorController = new CommandXboxController(Gamepads.OPERATOR);
      
    
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_dCtrl.button(1).onTrue(Commands.sequence(
      Commands.runOnce(() -> m_motors.setSparkSpeed(0.1)),
      Commands.print("cmd 1: run spark"),
      Commands.waitSeconds(2.0),
      Commands.runOnce(() -> m_motors.setSparkSpeed(0.0)),
      Commands.print("cmd 1: done")
    ));
  
    m_dCtrl.button(2).onTrue(Commands
      .runOnce(() -> m_motors.setTalonSpeed(0.3), m_motors)
      .andThen(Commands.print("cmd 2: on"))
    );
    m_dCtrl.button(2).onFalse(Commands
      .runOnce(() -> m_motors.setTalonSpeed(0.0), m_motors)
      .andThen(Commands.print("cmd 2: off"))
    );

    // This is a more compact way of doing two actions only on button down then up.
    m_dCtrl.button(3).whileTrue(Commands
      .startEnd(
        () -> {
          System.out.println("cmd 3: on");
          m_motors.setSparkSpeed(0.1);
        },
        () -> {
          System.out.println("cmd 3: off");
          m_motors.setSparkSpeed(0.0);
        },
        m_motors)
    );

    // Nice complex sequence.
    m_dCtrl.button(4).whileTrue(Commands.repeatingSequence(
      Commands.parallel(
        Commands.print("cmd 2: parallel"),
        Commands.runOnce(() -> m_motors.setSparkSpeed(0.1)),
        Commands.runOnce(() -> m_motors.setTalonSpeed(0.3))
      ),
      Commands.waitSeconds(3.0),
      Commands.runOnce(() -> {
        m_motors.setSparkSpeed(0.0);
        m_motors.setTalonSpeed(0.1);
      }),
      Commands.print("cmd 4: slower"),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> m_motors.setTalonSpeed(0.1)),
      Commands.print("cmd 4: just talon"),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> m_motors.setTalonSpeed(0.0)),
      Commands.print("cmd 4: paused"),
      Commands.waitSeconds(1.0)
    ).finallyDo(() -> {
      System.out.println("cmd 4: cleanup");
      m_motors.setTalonSpeed(0.0); 
      m_motors.setSparkSpeed(0.0);
    })
    );

    // m_dCtrl.a().onTrue(new InstantCommand(() -> m_LedSubsystem.setMode(modes.Green)));
    // m_dCtrl.b().onTrue(new InstantCommand(() -> m_LedSubsystem.setMode(modes.Blue)));
    // m_dCtrl.leftBumper().onTrue(new InstantCommand(() -> m_LedSubsystem.setMode(modes.oneSpace)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return Commands
      .print("Autonomous mode!")
      .andThen(Commands.print("jk we have no auto"));
  }
}
