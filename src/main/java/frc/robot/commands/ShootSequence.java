package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 

  private double m_speed; 

  public ShootSequence(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    switch (m_shooter.m_ScoringMode) {
            case AMP -> m_speed = m_shooter.ampSpeedSetpoint;
            case SPEAKER -> m_speed = m_shooter.speakerSpeedSetpoint; 
        };   

    addRequirements(m_shooter, m_intake, m_led);

    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Yellow)));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(-0.5)));
    this.addCommands(new WaitCommand(0.5));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(m_speed)));
    this.addCommands(new WaitCommand(1));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(1)));
    this.addCommands(new WaitCommand(1));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0)));
    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }
}
