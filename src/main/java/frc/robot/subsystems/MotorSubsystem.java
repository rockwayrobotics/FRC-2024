package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MotorSubsystem extends SubsystemBase{
  
    TalonSRX m_talon;
    CANSparkMax m_spark;

    ShuffleboardTab motors = Shuffleboard.getTab("Motors");

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable visiontable = inst.getTable("Vision");

    BooleanTopic vMotorTopic; 
    IntegerTopic vXTopic; 

    BooleanSubscriber vMotor;
    IntegerSubscriber vX; 


    public GenericEntry talonSpeed =
      motors.add("Talon Speed", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    public GenericEntry sparkSpeed =
      motors.add("Spark Speed", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();


    //---------------------------
    public MotorSubsystem() {
      // Initialize motor controllers
      m_talon = new TalonSRX(Constants.CAN.TALON_MOTOR);
      m_spark = new CANSparkMax(Constants.CAN.SPARK_MOTOR, MotorType.kBrushless); 

      vMotorTopic = visiontable.getBooleanTopic("motor");
      vXTopic = visiontable.getIntegerTopic("tag-x");

      vMotor = vMotorTopic.subscribe(false);
      vX = vXTopic.subscribe(0);
      
  }

  public void setTalonSpeed(double speed) {
    // Set the motor speed using the Talon SRX
    m_talon.set(TalonSRXControlMode.PercentOutput, speed); 
  }

  public void setSparkSpeed(double speed) {
    // Set the motor speed using the Spark MAX
    m_spark.set(speed); 
  }


  // public void periodic(){
  //   Boolean motorOn = vMotor.get();
  //   Integer xValue = (int) vX.get();

  //   if (motorOn){
  //     setTalonSpeed((xValue - 512.0) / 512);
  //   }
  //   else{
  //     setTalonSpeed(0);
  //   }
  // }
}