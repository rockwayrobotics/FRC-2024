package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_beltMotor;
  private final CANSparkMax m_leftIntake;
  private final CANSparkMax m_rightIntake;

  private DigitalInput m_intakeLoadSensor; 

  public boolean intakeLoad; 
  public boolean stagedFlag; 

  GenericEntry intakeLoadWidget; 

  RelativeEncoder m_beltEncoder;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Intake");

  public IntakeSubsystem(int beltMotor, int leftIntake, int rightIntake) {
    m_beltMotor = new CANSparkMax(beltMotor, CANSparkMax.MotorType.kBrushless);
    m_leftIntake = new CANSparkMax(leftIntake, CANSparkMax.MotorType.kBrushed);
    m_rightIntake = new CANSparkMax(rightIntake, CANSparkMax.MotorType.kBrushed);
    m_beltMotor.setIdleMode(IdleMode.kBrake);
    m_beltMotor.setInverted(true);
    m_leftIntake.setIdleMode(IdleMode.kCoast);
    m_rightIntake.setIdleMode(IdleMode.kCoast);

    m_rightIntake.follow(m_leftIntake, true);
    m_beltEncoder = m_beltMotor.getEncoder();

    // m_robotContainer = new RobotContainer();

    m_intakeLoadSensor = new DigitalInput(Constants.Digital.INTAKE_LOAD_SENSOR);
    intakeLoadWidget = dashboardTab.addPersistent("Intake Load", false).getEntry();
  }

  public void setBelt(double m_pow) {
    // System.out.println("Belt: " + m_pow);
    m_beltMotor.set(m_pow);
  }

  public void setIntake(double m_pow) {
    // System.out.println("Intake: " + m_pow);
    m_leftIntake.set(m_pow);
  }

  public boolean isNoteLoaded() {
    return !m_intakeLoadSensor.get();
  }

  public void periodic(){
    // if (!intakeLoad && isNoteLoaded()){
    //   new OperatorPullupSensor(m_robotContainer.m_shooter, m_robotContainer.m_intake, m_robotContainer.m_led)
    //   .withTimeout(1.5).andThen(new OperatorPullback(m_robotContainer.m_shooter, m_robotContainer.m_intake, m_robotContainer.m_led))
    //   .schedule();
    // }

    intakeLoadWidget.setBoolean(isNoteLoaded());

    intakeLoad = isNoteLoaded();
  }
}

