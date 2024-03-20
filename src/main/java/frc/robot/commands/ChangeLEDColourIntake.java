package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class ChangeLEDColourIntake extends Command {

  private LedSubsystem m_led;
  private IntakeSubsystem m_intake; 

  public ChangeLEDColourIntake(LedSubsystem led, IntakeSubsystem intake) {
    m_led = led;
    m_intake = intake;
    
    addRequirements(m_led, m_intake);
  }

  @Override
  public void initialize() {
    m_led.setMode(Constants.LED.modes.BreathingMagenta);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return !m_intake.isNoteLoaded(); 
  }

  @Override
  public void end(boolean cancelled) {
    m_led.setMode(Constants.LED.modes.Rainbow);
  }
}
