package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFlywheel extends Command {

    private final ShooterSubsystem m_shooter;
    private double m_speed;
    // private ScoringTarget targetAngle;
    // private ScoringTarget m_angle;

    public ShooterFlywheel(ShooterSubsystem shooter, double speed) {
        m_shooter = shooter;
        m_speed = speed;
    
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      m_shooter.setFlywheels(m_speed);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean cancelled) {
      m_shooter.setFlywheels(0);
    }
}
