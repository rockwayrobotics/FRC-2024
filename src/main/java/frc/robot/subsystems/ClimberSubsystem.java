package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_climberMotor;
  private final RelativeEncoder m_climberEncoder;

  private DigitalInput m_homeSensor;


  ShuffleboardTab dashboardTab = Shuffleboard.getTab("NewDashboard");

  GenericEntry homeSensorEntry;
  GenericEntry climberEncoderEntry;
  GenericEntry climberSpeedEntry; 
  GenericEntry softLimitEntry;

  /** Creates a new HookSubsystem. */
  public ClimberSubsystem(int climberMotor /* , int topLimitSwitch, int bottomLimitSwitch */) {
    m_climberMotor = new CANSparkMax(climberMotor, MotorType.kBrushless);
    m_climberMotor.setIdleMode(IdleMode.kBrake);
    m_climberMotor.setInverted(true);

    m_climberMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.FORWARD_MAX);
    m_climberMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.REVERSE_MAX);
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    m_climberEncoder = m_climberMotor.getEncoder();

    m_homeSensor = new DigitalInput(Constants.Digital.CLIMB_HOME_SWITCH);

    homeSensorEntry = 
    dashboardTab.addPersistent("Climb Home Sensor", false)
    .getEntry();

    climberEncoderEntry =
    dashboardTab.add("Climber Encoder", 0)
    .getEntry();

    climberSpeedEntry = 
    dashboardTab.add("Climber Speed", 0)
    .getEntry();

    softLimitEntry =
    dashboardTab.add("Climber Soft Limit", false)
    .getEntry();

  }

  public boolean isAtHome(){
    return !m_homeSensor.get();
  }

  /**
   * Extends the hook.
   */
  public void setClimber(double m_pow) {
    //System.out.println("Climber: " + m_pow);
    if (!isAtHome() || m_pow >= 0){
      m_climberMotor.set(m_pow);
    }
  }

  /**
   * Stops the hook.
   */
  public void stop() {
    m_climberMotor.set(0);
  }

  public void periodic() {
    if (isAtHome()) {
      if (m_climberMotor.get() < 0) {
        m_climberMotor.set(0);
      }
    }
    homeSensorEntry.setBoolean(isAtHome());
    climberEncoderEntry.setDouble(m_climberEncoder.getPosition());
    climberSpeedEntry.setDouble(m_climberMotor.get());
    boolean softLimitEnable = softLimitEntry.getBoolean(false);
      m_climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, softLimitEnable);
      m_climberMotor.enableSoftLimit(SoftLimitDirection.kForward, softLimitEnable);
    }
  }


