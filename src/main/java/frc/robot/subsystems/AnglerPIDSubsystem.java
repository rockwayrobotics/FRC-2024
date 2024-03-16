package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
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
  
  public GenericEntry speakerAngleWidget;
  public GenericEntry ampAngleWidget;
  public GenericEntry angleEncoderWidget;
  public GenericEntry angleSetpointWidget; 
  public GenericEntry outputWidget; 
  public GenericEntry errorWidget; 

  public GenericEntry kPWidget;
  public GenericEntry kIWidget;
  public GenericEntry kDWidget; 

  public GenericEntry positiveClampWidget;
  public GenericEntry negativeClampWidget;

  private double kPVal;
  private double kIVal;
  // private double kDVal; 

  private double positiveClamp;
  private double negativeClamp;

  // public double speakerAngleSetpoint;
  // public double ampAngleSetpoint;
  public double angleSetpoint;
  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("NewDashboard");


  public AnglerPIDSubsystem(){
    super(new PIDController(0.1, 0, 0));
    // getController().setTolerance(Constants.Angler.ANGLE_PID_TOLERANCE);

    m_angleMotor.setIdleMode(IdleMode.kBrake);

    m_angleMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Angler.ANGLE_BOTTOM_MAX);
    m_angleMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Angler.ANGLE_TOP_MAX);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    m_angleEncoder.setPosition(0);

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(0, 0).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(0, 0).getEntry();
    angleEncoderWidget = dashboardTab.addPersistent("Angle Encoder", 0).getEntry();
    angleSetpointWidget = dashboardTab.add("Angle Setpoint", 0).getEntry(); 
    outputWidget = dashboardTab.add("Output value", 0).getEntry();
    errorWidget = dashboardTab.add("PID Error Pos", 0).getEntry();

    kPWidget = dashboardTab.addPersistent("kP Value", 0.05).getEntry();
    kIWidget = dashboardTab.addPersistent("kI Value", 0).getEntry();
    // kDWidget = dashboardTab.addPersistent("kD Value", 0).getEntry();

    positiveClampWidget = dashboardTab.addPersistent("Positive Clamp", 0.05).getEntry();
    negativeClampWidget = dashboardTab.addPersistent("Negative Clamp", -0.1).getEntry();

    
    SmartDashboard.putData("Angle Encoder Reset", new InstantCommand(() -> resetAngleEncoder())); 
    SmartDashboard.putData("Put Back To Zero", new InstantCommand(() -> angleSetpointWidget.setDouble(0)));

    enable();
  }
  
  public void setScoringMode(ScoringMode mode) {
    m_ScoringMode = mode;
  }
  
  public void setAngleMotor(double speed) {
    // if (bottomShooterLimitPressed && Math.abs(speed) < 0){
    // m_angleMotor.set(0);
    // } else {

    //System.out.println("Angle: " + speed);

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
    outputWidget.setDouble(output);
    output = MathUtil.clamp(output, negativeClamp, positiveClamp);
    setAngleMotor(output);
    
    errorWidget.setDouble(getAngleEncoder() - setpoint);
    // System.out.println("calc: " + m_controller.calculate(getAngleEncoder(), setpoint));
    // System.out.println("Output: " + output);
    // System.out.println("Setpoint: " + setpoint);  
    }

  @Override
  protected double getMeasurement() {
    return getAngleEncoder();
  }

  @Override
  public void periodic(){
    // speakerAngleSetpoint = speakerAngleWidget.getDouble(0);
    // ampAngleSetpoint = ampAngleWidget.getDouble(0);
    kPVal = kPWidget.getDouble(0.05);
    kIVal = kIWidget.getDouble(0);
    getController().setP(kPVal);
    getController().setI(kIVal);

    angleSetpoint = angleSetpointWidget.getDouble(0);
    setSetpoint(angleSetpoint);

    positiveClamp = positiveClampWidget.getDouble(0.05);
    negativeClamp = negativeClampWidget.getDouble(-0.1);
    
    super.periodic();

    angleEncoderWidget.setDouble(getAngleEncoder());
  }
}

