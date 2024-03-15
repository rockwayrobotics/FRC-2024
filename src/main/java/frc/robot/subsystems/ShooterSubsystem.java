package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.Shooter.ScoringMode;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_angleMotor;
  private final CANSparkMax m_leftFlywheel;
  private final CANSparkMax m_rightFlywheel;

  private DigitalInput m_shooterTopSensor; 
  private DigitalInput m_intakeLoadSensor; 

  RelativeEncoder m_angleEncoder;

  public double speakerAngleSetpoint;
  public double ampAngleSetpoint; 
  public boolean shooterStaged;
  public boolean intakeLoad; 
  
  GenericEntry speakerAngleWidget;
  GenericEntry ampAngleWidget;
  GenericEntry shooterTopSensorWidget; 
  GenericEntry intakeLoadWidget; 
  GenericEntry angleEncoderWidget;

  private double m_scale = 1;

  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("NewDashboard");

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

    m_angleMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Shooter.ANGLE_BOTTOM_MAX);
    m_angleMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Shooter.ANGLE_TOP_MAX);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    m_rightFlywheel.follow(m_leftFlywheel, true);
    m_angleEncoder = m_angleMotor.getEncoder();

    m_angleEncoder.setPosition(0);

    m_shooterTopSensor = new DigitalInput(Constants.Digital.SHOOTER_TOP_SENSOR);
    m_intakeLoadSensor = new DigitalInput(Constants.Digital.INTAKE_LOAD_SENSOR);

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(0, 0).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(0, 0).getEntry();
    shooterTopSensorWidget = dashboardTab.addPersistent("Shooter Staged Sensor", false).getEntry();
    intakeLoadWidget = dashboardTab.addPersistent("Intake Load", false).getEntry();
    angleEncoderWidget = dashboardTab.addPersistent("Angle Encoder", 0).getEntry();

    SmartDashboard.putData("Angle Encoder Reset", new InstantCommand(() -> resetAngleEncoder())); 
  }

  public void setFlywheelsScale(double scale) {
    m_scale = scale;
  }

  public void setFlywheels(double m_pow) {
    // System.out.println("Flywheels: " + m_pow);
    m_leftFlywheel.set(m_pow * m_scale);
  }

  public void setAngleMotor(double speed) {
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

  public void resetAngleEncoder(){
    m_angleEncoder.setPosition(0);
  }

  public void setScoringMode(ScoringMode mode) {
    m_ScoringMode = mode;
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

    speakerAngleSetpoint = speakerAngleWidget.getDouble(0);
    ampAngleSetpoint = ampAngleWidget.getDouble(0);
    shooterTopSensorWidget.setBoolean(isNoteStaged());
    intakeLoadWidget.setBoolean(isNoteLoaded());
    angleEncoderWidget.setDouble(getAngleEncoder());

    shooterStaged = isNoteStaged();
    intakeLoad = isNoteLoaded();
  }
}
