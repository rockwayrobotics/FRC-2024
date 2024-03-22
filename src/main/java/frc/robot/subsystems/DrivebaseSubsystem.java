package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class DrivebaseSubsystem extends SubsystemBase {

  CANSparkMax m_leftDriveMotorF;
  CANSparkMax m_leftDriveMotorR;
  CANSparkMax m_rightDriveMotorF;
  CANSparkMax m_rightDriveMotorR;

  private final DifferentialDrive m_drive;

  RelativeEncoder m_leftDriveEncoder;
  RelativeEncoder m_rightDriveEncoder;

  SlewRateLimiter filter = new SlewRateLimiter(0);

  private double m_scale = 1;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double yawOffset;
  private DifferentialDriveOdometry driveOdometry; 

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

    driveOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLDistance(), getRDistance());

    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void setDrivebaseIdle(IdleMode setting) {
    m_rightDriveMotorF.setIdleMode(setting);
    m_rightDriveMotorR.setIdleMode(setting);
    m_leftDriveMotorF.setIdleMode(setting);
    m_leftDriveMotorR.setIdleMode(setting);
  }

  public void disable(){
    setDrivebaseIdle(IdleMode.kCoast);
  }

  public void stop() {
    set(0, 0);
  }

  public void set(double speed, double rotation) {
    // speed = filter.calculate(speed);
    m_drive.curvatureDrive(speed * m_scale, rotation * m_scale, true);
  }

  public void setScale(double scale) {
    m_scale = scale;
  }

  // Get the pose of the robot as Pose2d 
  public Pose2d getPose(){
    return driveOdometry.getPoseMeters();
  }


  // Reset the Pose2d of the robot 
  public void resetPose(Pose2d pose2d) {
    this.resetEncoders();
    this.driveOdometry.resetPosition(
        m_gyro.getRotation2d(),
        getLDistance(),
        getRDistance(),
        pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return new ChassisSpeeds(getLDistance(), getRDistance(), getAngle());
  }

  public void drive(ChassisSpeeds speeds){
    set(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since
   * last reset.
   * 
   * @return Distance, in inches.
   */
  public double getLDistance() {
    return m_leftDriveEncoder.getPosition();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since
   * last reset.
   * 
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
  public boolean resetEncoders() {
   return m_leftDriveEncoder.setPosition(0) == REVLibError.kOk &&
    m_rightDriveEncoder.setPosition(0) == REVLibError.kOk;
  }
}
