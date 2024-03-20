package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequenceFull extends ParallelCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 


  private ShootSequenceFull(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_shooter, m_intake, m_led);
    Command main = Commands.sequence(
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.Yellow)),
      Commands.runOnce(() -> m_shooter.setFlywheels(1)),
      Commands.waitSeconds(0.8),
      Commands.runOnce(() -> m_intake.setBelt(1)),
      Commands.waitSeconds(1),
      Commands.runOnce(() -> {
        m_intake.setBelt(0);
        m_shooter.setFlywheels(0);
      }),
      Commands.print("sequence complete")
    );
    Command LEDindicator = Commands.sequence(
      Commands.waitUntil(() -> m_shooter.isNoteStaged()),
      Commands.waitSeconds(0.2),
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.whiteDotLines)),
      Commands.waitSeconds(0.2),
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.Off)),
      Commands.waitSeconds(0.2),
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.whiteDotLines)),
      Commands.waitSeconds(0.2),
      Commands.print("setting back to rainbow"),
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.Rainbow))
    );

    this.addCommands(main, LEDindicator);
  }

  public static Command create(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    return new ShootSequenceFull(shooter, intake, led).finallyDo((boolean interrupted) -> {
      shooter.setFlywheels(0);
    });
  }
}
