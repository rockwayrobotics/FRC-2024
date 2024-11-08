package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants;
import frc.robot.Constants.Drive;

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

  public double distanceDrivenAuto;
  public double rotationScale;
  boolean isBrakeMode;
  boolean isCoastMode;

  GenericEntry rotationScaleWidget;
  GenericEntry brakeModeWidget;
  GenericEntry coastModeWidget;
  GenericEntry leftDistanceWidget;
  GenericEntry rightDistanceWidget;

  GenericEntry navxMonitorWidget;
  GenericEntry counterWidget;

  Field2d field = new Field2d();
  // GenericEntry fieldWidget;

  double counter = 0;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double yawOffset;
  private DifferentialDriveOdometry driveOdometry;

  // Tracking of whether we have encoder reset issues.
  private int leftErrorCount = 0;
  private int rightErrorCount = 0;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Drivebase");

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

    m_leftDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.LEFT_SCALING);
    m_rightDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.RIGHT_SCALING);

    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
    
    rotationScaleWidget = dashboardTab.addPersistent("Driving Rotation Scale Factor", 0.76)
    .getEntry();

    brakeModeWidget = dashboardTab.add("Brake Mode", false).getEntry();
    coastModeWidget = dashboardTab.add("Coast Mode", false).getEntry();

    navxMonitorWidget = dashboardTab.add("NavX Monitor", 0).getEntry();
    counterWidget = dashboardTab.add("Counter Widget", 0).getEntry();

    leftDistanceWidget = dashboardTab.add("Left Distance", 0).getEntry();
    rightDistanceWidget = dashboardTab.add("Right Distance", 0).getEntry();

    // Note: the dashboard listens to changes on the field object
    // so we don't have to publish changes explicitly.
    dashboardTab.add("Field2d", field);

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
    setDrivebaseIdle(IdleMode.kBrake);
  }

  public void stop() {
    set(0, 0);
  }

  public void set(double speed, double rotation) {
    // speed = filter.calculate(speed);
    m_drive.curvatureDrive(speed * m_scale, rotation * m_scale, true);
  }

  public void setPathPlannerSpeed(double speed, double rotation) {
    speed = MathUtil.clamp(speed, -0.4, 0.4);
    rotation = MathUtil.clamp(rotation, -0.4, 0.4);
    m_drive.curvatureDrive(speed, rotation, true);
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
    System.out.printf("Resetting pose to %s%n", pose2d.toString());
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return new ChassisSpeeds(m_leftDriveEncoder.getVelocity(), m_rightDriveEncoder.getVelocity(), getAngle());
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
  public void resetEncoders() {
    // Troubleshooting for theorized CAN failure where our attempt
    // to reset an encoder fails.
    // We're trying both (before checking failures, so we can
    // track whether just one or both fail), and returning
    // if all is well, but otherwise counting the failures and
    // retrying.  We'll print the stats as we change states.
    for (int i = 0; i < 5; i++) {
      // Reset but check that return code is good.  In a future world
      // we can do more like recording the specific error code, but
      // since we're adding this mid-competition I want to be conservative.
      boolean leftOk = m_leftDriveEncoder.setPosition(0) == REVLibError.kOk;
      boolean rightOk = m_rightDriveEncoder.setPosition(0) == REVLibError.kOk;

      if (leftOk && rightOk) {
          break;  // Success! Get us out of here.
      } else {
          if (!leftOk)
              leftErrorCount += 1;
          if (!rightOk)
              rightErrorCount += 1;
      }
    }
  }

  public void reportFailures(String state) {
    System.out.printf("Reset failures (%s): %d, %d%n",
      state, leftErrorCount, rightErrorCount);
  }

  @Override
  public void periodic(){
    driveOdometry.update(m_gyro.getRotation2d(), getLDistance(), getRDistance());
    rotationScale = rotationScaleWidget.getDouble(0.76);
    leftDistanceWidget.setDouble(getLDistance());
    rightDistanceWidget.setDouble(getRDistance());

    // This publishes changes through to the dashboard.
    field.setRobotPose(driveOdometry.getPoseMeters());

    if (isBrakeMode != brakeModeWidget.getBoolean(false)){
      isBrakeMode = brakeModeWidget.getBoolean(false);
      if (isBrakeMode){
        setDrivebaseIdle(IdleMode.kBrake);
      }

    if (isCoastMode != coastModeWidget.getBoolean(false)){
      isCoastMode = coastModeWidget.getBoolean(false);
      if (isCoastMode){
        setDrivebaseIdle(IdleMode.kCoast);
      }
      }
    }
      navxMonitorWidget.setDouble(m_gyro.getAngle());

      counter++;

      if (counter == 50){
        counter = 0;
      }
      counterWidget.setDouble(counter);
  }
}
