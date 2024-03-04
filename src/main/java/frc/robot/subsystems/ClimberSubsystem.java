package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_climberMotor;

  private double m_pow = 0;

  /** Creates a new HookSubsystem. */
  public ClimberSubsystem(int climberMotor /*, int topLimitSwitch, int bottomLimitSwitch*/) {
    m_climberMotor = new CANSparkMax(climberMotor, MotorType.kBrushless);
    m_climberMotor.setIdleMode(IdleMode.kBrake);

    // m_topLimitSwitch =  new DigitalInput(topLimitSwitch);
    // m_bottomLimitSwitch = new DigitalInput(bottomLimitSwitch);
  }
  
  /**
   * Extends the hook.
   */
  public void extend(double extendPow) {

    // For some reason switch is reading inverse, false when pushed down and true when not pushed

    // if(!m_topLimitSwitch.get()) {
    //   m_pow = 0;
    // } else {
      m_pow = extendPow;
    // }
  }

  /**
   * Retracts the hook.
   */
  public void retract(double retractPow) {
    // if(!m_bottomLimitSwitch.get()) {
    //   m_pow = 0;
    // } else {
      m_pow = retractPow;
    // }
  }

  @Override
  public void periodic() {
    // if either limit switch is pressed (represented by false), set power to zero
    // if(m_pow > 0 && !m_topLimitSwitch.get() || m_pow < 0 && !m_bottomLimitSwitch.get()) {
      m_pow = 0;
    // }
    m_climberMotor.set(m_pow);
  }
}
