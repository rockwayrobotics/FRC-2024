package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DrivebaseSubsystem extends SubsystemBase {

    CANSparkMax m_leftDriveMotor1;
    CANSparkMax m_leftDriveMotor2;
    CANSparkMax m_rightDriveMotor1;
    CANSparkMax m_rightDriveMotor2; 

    private final DifferentialDrive m_drive;

    private final Encoder m_leftDriveEncoder;
    private final Encoder m_rightDriveEncoder;

    private double m_scale = 1;

    public DrivebaseSubsystem () {
        m_leftDriveMotor1 = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
        m_leftDriveMotor1.restoreFactoryDefaults();
        m_leftDriveMotor1.setSmartCurrentLimit(38);

        m_leftDriveMotor2 = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);
        m_leftDriveMotor2.restoreFactoryDefaults();
        m_leftDriveMotor2.setSmartCurrentLimit(38);

        m_rightDriveMotor1 = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
        m_rightDriveMotor1.restoreFactoryDefaults();
        m_rightDriveMotor1.setSmartCurrentLimit(38);

        m_rightDriveMotor2 = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
        m_rightDriveMotor2.restoreFactoryDefaults();
        m_rightDriveMotor2.setSmartCurrentLimit(38);

        m_leftDriveMotor2.follow(m_leftDriveMotor1);
        m_leftDriveMotor1.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);
        m_rightDriveMotor2.follow(m_rightDriveMotor2);
        m_rightDriveMotor1.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

        m_drive = new DifferentialDrive(m_leftDriveMotor1, m_rightDriveMotor1);

        setDrivebaseIdle(IdleMode.kBrake);
        m_leftDriveEncoder = new Encoder(Constants.Digital.LEFT_DRIVE_ENCODER[0], Constants.Digital.LEFT_DRIVE_ENCODER[1]);
        m_rightDriveEncoder = new Encoder(Constants.Digital.RIGHT_DRIVE_ENCODER[0], Constants.Digital.RIGHT_DRIVE_ENCODER[1]);
        // when robot goes forward, left encoder spins positive and right encoder spins negative
        m_leftDriveEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
        m_rightDriveEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
        m_leftDriveEncoder.setReverseDirection(Constants.Drive.LEFT_DRIVE_INVERTED);
        m_rightDriveEncoder.setReverseDirection(Constants.Drive.RIGHT_DRIVE_INVERTED);
        m_leftDriveEncoder.reset();
        m_rightDriveEncoder.reset();

    }
    
    public void setDrivebaseIdle(IdleMode setting) {
        m_rightDriveMotor1.setIdleMode(setting);
        m_rightDriveMotor2.setIdleMode(setting);
        m_leftDriveMotor1.setIdleMode(setting);
        m_leftDriveMotor2.setIdleMode(setting);
    }

    public void stop(){
        set(0,0);
    }

    public void set(double speed, double rotation) {
        m_drive.curvatureDrive(speed*m_scale, rotation*m_scale, true);
    }

    public void setScale(double scale) {
        m_scale = scale;
    }
}
