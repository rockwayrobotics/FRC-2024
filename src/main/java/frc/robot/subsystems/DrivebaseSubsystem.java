package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DrivebaseSubsystem extends SubsystemBase {

  CANSparkMax m_leftDriveMotorF;
  CANSparkMax m_leftDriveMotorR;
  CANSparkMax m_rightDriveMotorF;
  CANSparkMax m_rightDriveMotorR;

  private final DifferentialDrive m_drive;

  private final Encoder m_leftDriveEncoder;
  private final Encoder m_rightDriveEncoder;

  private double m_scale = 1;

  public DrivebaseSubsystem() {
    m_leftDriveMotorF = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_F, MotorType.kBrushless);
    m_leftDriveMotorF.restoreFactoryDefaults();
    m_leftDriveMotorF.setSmartCurrentLimit(38);

    m_leftDriveMotorR = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_R, MotorType.kBrushless);
    m_leftDriveMotorR.restoreFactoryDefaults();
    m_leftDriveMotorR.setSmartCurrentLimit(38);

    m_rightDriveMotorF = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_F, MotorType.kBrushless);
    m_rightDriveMotorF.restoreFactoryDefaults();
    m_rightDriveMotorF.setSmartCurrentLimit(38);

    m_rightDriveMotorR = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_R, MotorType.kBrushless);
    m_rightDriveMotorR.restoreFactoryDefaults();
    m_rightDriveMotorR.setSmartCurrentLimit(38);

    m_leftDriveMotorR.follow(m_leftDriveMotorF);
    m_leftDriveMotorF.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);
    m_rightDriveMotorR.follow(m_rightDriveMotorF);
    m_rightDriveMotorF.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

    m_drive = new DifferentialDrive(m_leftDriveMotorF, m_rightDriveMotorF);

    setDrivebaseIdle(IdleMode.kBrake);
    m_leftDriveEncoder = new Encoder(Constants.Digital.LEFT_DRIVE_ENCODER[0],
        Constants.Digital.LEFT_DRIVE_ENCODER[1]);
    m_rightDriveEncoder = new Encoder(Constants.Digital.RIGHT_DRIVE_ENCODER[0],
        Constants.Digital.RIGHT_DRIVE_ENCODER[1]);
    // when robot goes forward, left encoder spins positive and right encoder spins
    // negative
    m_leftDriveEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightDriveEncoder.setDistancePerPulse(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
    m_leftDriveEncoder.setReverseDirection(Constants.Drive.LEFT_DRIVE_INVERTED);
    m_rightDriveEncoder.setReverseDirection(Constants.Drive.RIGHT_DRIVE_INVERTED);
    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();

  }

  public void setDrivebaseIdle(IdleMode setting) {
    m_rightDriveMotorF.setIdleMode(setting);
    m_rightDriveMotorR.setIdleMode(setting);
    m_leftDriveMotorF.setIdleMode(setting);
    m_leftDriveMotorR.setIdleMode(setting);
  }

  public void stop() {
    set(0, 0);
  }

  public void set(double speed, double rotation) {
    m_drive.curvatureDrive(speed * m_scale, rotation * m_scale, true);
  }

  public void setScale(double scale) {
    m_scale = scale;
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since last reset.
   * @return Distance, in inches.
   */
  public double getLDistance() {
    return m_leftDriveEncoder.getDistance();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since last reset.
   * @return Distance in inches.
   */
  public double getRDistance() {
    return m_rightDriveEncoder.getDistance();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();
  }
}
