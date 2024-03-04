package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.Shooter.ScoringMode;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_angleMotor;
  private final CANSparkMax m_flywheelLeft;
  private final CANSparkMax m_flywheelRight;

  RelativeEncoder m_angleEncoder;

  public double speakerAngleSetpoint;
  public double ampAngleSetpoint;
  GenericEntry speakerAngleWidget;
  GenericEntry ampAngleWidget;

  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;
  

  /** Creates a new HookSubsystem. */
  public ShooterSubsystem(int angleMotor, int flywheelLeft, int flywheelRight) {
    m_angleMotor = new CANSparkMax(Constants.CAN.GEAR, MotorType.kBrushless);
    m_flywheelLeft = new CANSparkMax(Constants.CAN.LEFT_FLYWHEEL, MotorType.kBrushless);
    m_flywheelRight = new CANSparkMax(Constants.CAN.RIGHT_FLYWHEEL, MotorType.kBrushless);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_flywheelLeft.setIdleMode(IdleMode.kCoast);
    m_flywheelRight.setIdleMode(IdleMode.kCoast);

    m_angleEncoder = m_angleMotor.getEncoder();

    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dashboard");

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(2,2).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(3,2).getEntry();
  }

  public void setFlywheels(double m_pow) {
    m_flywheelLeft.set(m_pow);
    m_flywheelRight.set(-m_pow);
  }

  public void spinAngleMotor(double speed) {
    // if (bottomShooterLimitPressed && Math.abs(speed) < 0){
    //   m_angleMotor.set(0);
    // } else {
      m_angleMotor.set(speed);
    // }
  }

  public double getAngleEncoder() {
    return m_angleEncoder.getPosition();
  }

  public void setScoringMode(ScoringMode mode) {
    m_ScoringMode = mode;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder revolutions", getAngleEncoder());

    speakerAngleSetpoint = speakerAngleWidget.getDouble(0);
    ampAngleSetpoint = ampAngleWidget.getDouble(0);
  }
}
