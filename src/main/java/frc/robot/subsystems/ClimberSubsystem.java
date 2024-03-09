package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_climberMotor;

  /** Creates a new HookSubsystem. */
  public ClimberSubsystem(int climberMotor /* , int topLimitSwitch, int bottomLimitSwitch */) {
    m_climberMotor = new CANSparkMax(climberMotor, MotorType.kBrushless);
    m_climberMotor.setIdleMode(IdleMode.kBrake);
    m_climberMotor.setInverted(true);

    // m_topLimitSwitch = new DigitalInput(topLimitSwitch);
    // m_bottomLimitSwitch = new DigitalInput(bottomLimitSwitch);
  }

  /**
   * Extends the hook.
   */
  public void setClimber(double m_pow) {
    System.out.println("Climber: " + m_pow);
    m_climberMotor.set(m_pow);
    
  }

  /**
   * Stops the hook.
   */
  public void stop() {
    m_climberMotor.set(0);
  }
}
