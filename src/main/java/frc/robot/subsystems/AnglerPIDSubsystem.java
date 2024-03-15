package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.ScoringMode;

public class AnglerPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax m_angleMotor = new CANSparkMax(Constants.CAN.GEAR, MotorType.kBrushless);

  RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
  
  GenericEntry speakerAngleWidget;
  GenericEntry ampAngleWidget;
  GenericEntry angleEncoderWidget;

  public double speakerAngleSetpoint;
  public double ampAngleSetpoint; 
  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("NewDashboard");


  public AnglerPIDSubsystem(){
    super(new PIDController(0.2, 0, 0.1));
    getController().setTolerance(Constants.Angler.ANGLE_PID_TOLERANCE);
    setSetpoint(Constants.Angler.SPEAKER_SETPOINT);

    m_angleMotor.setIdleMode(IdleMode.kBrake);

    m_angleMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Angler.ANGLE_BOTTOM_MAX);
    m_angleMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Angler.ANGLE_TOP_MAX);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    m_angleEncoder.setPosition(0);

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(0, 0).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(0, 0).getEntry();
    angleEncoderWidget = dashboardTab.addPersistent("Angle Encoder", 0).getEntry();
    
    SmartDashboard.putData("Angle Encoder Reset", new InstantCommand(() -> resetAngleEncoder())); 
  }
  
  public void setScoringMode(ScoringMode mode) {
    m_ScoringMode = mode;
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

  @Override
  protected void useOutput(double output, double setpoint) {
    //setAngleMotor(m_controller.calculate(getAngleEncoder(), setpoint));
    System.out.println(m_controller.calculate(getAngleEncoder(), setpoint));
    System.out.println("Output: " + output);
    System.out.println("Setpoint: " + setpoint);  
    }

  @Override
  protected double getMeasurement() {
    return getAngleEncoder();
  }

  @Override
  public void periodic(){
    speakerAngleSetpoint = speakerAngleWidget.getDouble(0);
    ampAngleSetpoint = ampAngleWidget.getDouble(0);
    angleEncoderWidget.setDouble(getAngleEncoder());
  }
}

