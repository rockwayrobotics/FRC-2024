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
  private final CANSparkMax m_leftFlywheel;
  private final CANSparkMax m_rightFlywheel;

  RelativeEncoder m_angleEncoder;

  public double speakerAngleSetpoint;
  public double ampAngleSetpoint;
  public double trapAngleSetpoint;
  
  GenericEntry speakerAngleWidget;
  GenericEntry ampAngleWidget;

  private double m_scale = 1;

  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;

  /** Creates a new HookSubsystem. */
  public ShooterSubsystem(int angleMotor, int leftFlywheel, int rightFlywheel) {
    m_angleMotor = new CANSparkMax(Constants.CAN.GEAR, MotorType.kBrushless);
    m_leftFlywheel = new CANSparkMax(Constants.CAN.LEFT_FLYWHEEL, MotorType.kBrushless);
    m_rightFlywheel = new CANSparkMax(Constants.CAN.RIGHT_FLYWHEEL, MotorType.kBrushless);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_leftFlywheel.setIdleMode(IdleMode.kCoast);
    m_leftFlywheel.setInverted(true);
    m_rightFlywheel.setIdleMode(IdleMode.kCoast);

    m_leftFlywheel.setSmartCurrentLimit(40);
    m_rightFlywheel.setSmartCurrentLimit(40);

    m_rightFlywheel.follow(m_leftFlywheel, true);
    m_angleEncoder = m_angleMotor.getEncoder();

    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dashboard");

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(0, 0).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(0, 0).getEntry();
  }

  public void setFlywheelsScale(double scale) {
    m_scale = scale;
  }

  public void setFlywheels(double m_pow) {
    // System.out.println("Flywheels: " + m_pow);
    m_leftFlywheel.set(m_pow * m_scale);
  }

  public void spinAngleMotor(double speed) {
    // if (bottomShooterLimitPressed && Math.abs(speed) < 0){
    // m_angleMotor.set(0);
    // } else {
    System.out.println("Angle: " + speed);
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
