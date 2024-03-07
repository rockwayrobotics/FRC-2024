package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.Constants;
//import frc.robot.subsystems.LedSubsystem;

public class LoadShooterSequence extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;

  // LedSubsystem m_led;

  public LoadShooterSequence(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_intake = intake;
    // m_led = led;
    addRequirements(m_intake, m_shooter);

    // this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Red)));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0.5)));
    this.addCommands(new WaitCommand(3));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(-0.5)));
    this.addCommands(new WaitCommand(0.5));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));    
  }
}
