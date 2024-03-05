package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DrivebaseSubsystem extends SubsystemBase {

  CANSparkMax m_leftDriveMotorF;
  CANSparkMax m_leftDriveMotorR;
  CANSparkMax m_rightDriveMotorF;
  CANSparkMax m_rightDriveMotorR;

  private final DifferentialDrive m_drive;

  RelativeEncoder m_leftDriveEncoder;
  RelativeEncoder m_rightDriveEncoder;

  private double m_scale = 1;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double yawOffset;

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
    m_leftDriveEncoder = m_leftDriveMotorF.getEncoder();
    m_rightDriveEncoder = m_rightDriveMotorF.getEncoder();
    // when robot goes forward, left encoder spins positive and right encoder spins
    // negative
    m_leftDriveEncoder.setPositionConversionFactor(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightDriveEncoder.setPositionConversionFactor(Constants.Drive.DISTANCE_PER_ENCODER_PULSE);
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
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
    return m_leftDriveEncoder.getPosition();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since last reset.
   * @return Distance in inches.
   */
  public double getRDistance() {
    return m_rightDriveEncoder.getPosition();
  }

  public void zeroGyro() {
    System.out.println("NavX Connected: " + m_gyro.isConnected());
    m_gyro.reset();
  }
  public void setAutoOffset(double autoOffset) {
    yawOffset = autoOffset;
  }
  public double getYaw() {
    return m_gyro.getYaw() + yawOffset;
  }
  public double getPitch() {
    return m_gyro.getPitch();
  }
  public double getRoll() {
    return m_gyro.getRoll();
  }
  public double getAngle() {
    return m_gyro.getAngle();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
  }
}
