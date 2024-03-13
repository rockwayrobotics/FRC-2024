package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_beltMotor;
  private final CANSparkMax m_leftIntake;
  private final CANSparkMax m_rightIntake;

  RelativeEncoder m_beltEncoder;

  public IntakeSubsystem(int beltMotor, int leftIntake, int rightIntake) {
    m_beltMotor = new CANSparkMax(beltMotor, CANSparkMax.MotorType.kBrushless);
    m_leftIntake = new CANSparkMax(leftIntake, CANSparkMax.MotorType.kBrushed);
    m_rightIntake =
      new CANSparkMax(rightIntake, CANSparkMax.MotorType.kBrushed);
    m_beltMotor.setIdleMode(IdleMode.kBrake);
    m_beltMotor.setInverted(true);
    m_leftIntake.setIdleMode(IdleMode.kCoast);
    m_rightIntake.setIdleMode(IdleMode.kCoast);

    m_rightIntake.follow(m_leftIntake, true);
    m_beltEncoder = m_beltMotor.getEncoder();
  }

  public void setBelt(double m_pow) {
    // System.out.println("Belt: " + m_pow);
    m_beltMotor.set(m_pow);
  }

  public void setIntake(double m_pow) {
    // System.out.println("Intake: " + m_pow);
    m_leftIntake.set(m_pow);
  }
}
