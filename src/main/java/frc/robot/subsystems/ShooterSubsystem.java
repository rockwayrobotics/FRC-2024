package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftFlywheel;
  private final CANSparkMax m_rightFlywheel;

  private DigitalInput m_shooterTopSensor; 
  private DigitalInput m_intakeLoadSensor; 

  public boolean shooterStaged;
  public boolean intakeLoad; 
  
  GenericEntry shooterTopSensorWidget; 
  GenericEntry intakeLoadWidget; 

  private double m_scale = 1;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("NewDashboard");

  /** Creates a new HookSubsystem. */
  public ShooterSubsystem(int leftFlywheel, int rightFlywheel) {
    m_leftFlywheel = new CANSparkMax(Constants.CAN.LEFT_FLYWHEEL, MotorType.kBrushless);
    m_rightFlywheel = new CANSparkMax(Constants.CAN.RIGHT_FLYWHEEL, MotorType.kBrushless);
    m_leftFlywheel.setIdleMode(IdleMode.kCoast);
    m_leftFlywheel.setInverted(true);
    m_rightFlywheel.setIdleMode(IdleMode.kCoast);

    m_leftFlywheel.setSmartCurrentLimit(40);
    m_rightFlywheel.setSmartCurrentLimit(40);

    m_rightFlywheel.follow(m_leftFlywheel, true);

    m_shooterTopSensor = new DigitalInput(Constants.Digital.SHOOTER_TOP_SENSOR);
    m_intakeLoadSensor = new DigitalInput(Constants.Digital.INTAKE_LOAD_SENSOR);

    shooterTopSensorWidget = dashboardTab.addPersistent("Shooter Staged Sensor", false).getEntry();
    intakeLoadWidget = dashboardTab.addPersistent("Intake Load", false).getEntry();
  }

  public void setFlywheelsScale(double scale) {
    m_scale = scale;
  }

  public void setFlywheels(double m_pow) {
    // System.out.println("Flywheels: " + m_pow);
    m_leftFlywheel.set(m_pow * m_scale);
  }

  public boolean isNoteStaged() {
    return !m_shooterTopSensor.get();
  }

  public boolean isNoteLoaded() {
    return !m_intakeLoadSensor.get();
  }

  @Override
  public void periodic() {

    //SmartDashboard.putNumber("Encoder revolutions", getAngleEncoder());

    // this one broke some stuff, look at it later 
    //dashboardTab.add("Encoder revolutions", getAngleEncoder());

    shooterTopSensorWidget.setBoolean(isNoteStaged());
    intakeLoadWidget.setBoolean(isNoteLoaded());

    shooterStaged = isNoteStaged();
    intakeLoad = isNoteLoaded();
  }
}
