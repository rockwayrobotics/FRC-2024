package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MotorSubsystem extends SubsystemBase{
  
  TalonSRX m_talon;
  CANSparkMax m_spark;

/** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    // Initialize motor controllers
    m_talon = new TalonSRX(Constants.CAN.TALON_MOTOR);
    m_spark = new CANSparkMax(Constants.CAN.SPARK_MOTOR, MotorType.kBrushless); 
}

public void setTalonSpeed(double speed) {
  // Set the motor speed using the Talon SRX
  m_talon.set(TalonSRXControlMode.PercentOutput, speed); 
}

public void setSparkSpeed(double speed) {
  // Set the motor speed using the Talon SRX
  m_spark.set(speed); 
}

}
