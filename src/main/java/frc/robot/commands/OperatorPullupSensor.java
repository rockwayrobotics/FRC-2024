package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
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
    m_intake.setBelt(1);
  }

  @Override
  public void execute() {
    System.out.println("Executing Operator Pullup");
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isNoteStaged(); // Returns true if the shooter is staged
  }

  @Override
  public void end(boolean cancelled) {
    m_intake.setBelt(-0.5);
    new WaitCommand(0.2);
    m_intake.setBelt(0);
  }
}
