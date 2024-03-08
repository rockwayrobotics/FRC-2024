package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequenceHalf extends SequentialCommandGroup{
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake; 

  public ShootSequenceHalf(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(m_shooter, m_intake);

  
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0.5)));
    this.addCommands(new WaitCommand(0.5));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0.5)));
    this.addCommands(new WaitCommand(1));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0)));  
  }
}
