package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorPullupSensor extends Command {

  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led;

  public OperatorPullupSensor(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_shooter, m_intake, m_led);
  }

  @Override
  public void initialize() {
    // Resets encoder values to default
    //System.out.println("operator pullup");
    m_intake.setBelt(0.7);
    m_led.setMode(Constants.LED.modes.FlashingOrange);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isNoteStaged(); // Returns true if the shooter is staged
  }

  @Override
  public void end(boolean cancelled) {
    // m_intake.setBelt(0);
    //System.out.println(cancelled);
  }
}
