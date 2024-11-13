package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
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
  DifferentialDriveKinematics m_kinematics;

  RelativeEncoder m_leftDriveEncoder;
  RelativeEncoder m_rightDriveEncoder;

  SlewRateLimiter filter = new SlewRateLimiter(0);

  private double m_scale = 1;

  private DifferentialDrivetrainSim m_drivetrainSim;

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

  // Turn on LTV path building instead of ramsete.
  private boolean m_experimentalLTV = false;

  /////////////////////////////
  // Simulation variables - would be nice to remove or minimize these to ensure there
  // is no impact on non-simulation performance.
  private boolean m_isSimulation = false;

  /** Holds the last simulation time for duration calculation, matches REVPhysicsSim */
  private long m_lastSimTime;

  /** Allows first-time initialization of last simulation time. */
  private boolean m_startedSimulation = false;
  /////////////////////////////

  public DrivebaseSubsystem(boolean isSimulation) {
    m_isSimulation = isSimulation;

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

    // Position is measured in motor revolutions by default, but we want metres.
    // Wheel encoder scaling gives us centimetres and takes the gear ratio into account,
    // and the scaling values appear to take more accurate wheel measurements and cm -> m
    m_leftDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.LEFT_SCALING);
    m_rightDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.RIGHT_SCALING);

    // Distance conversion is the same as the position factor, but convert from minutes to seconds
    // since velocity is measured in rpm by default but we want m/s.
    m_leftDriveEncoder.setVelocityConversionFactor(m_leftDriveEncoder.getPositionConversionFactor() / 60);
    m_rightDriveEncoder.setVelocityConversionFactor(m_rightDriveEncoder.getPositionConversionFactor() / 60);

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
    m_kinematics = new DifferentialDriveKinematics(0.56);
    driveOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d().unaryMinus(), getLDistance(), getRDistance());

    PathPlannerLogging.setLogActivePathCallback(this::saveActivePath);
    PathPlannerLogging.setLogTargetPoseCallback(this::saveTargetPose);

    if (m_experimentalLTV) {
      AutoBuilder.configureLTV(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            m_isSimulation ? this::getSimulationSpeeds : this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            0.02, // duration in seconds between update loop calls, defaults to 0.02s = 20ms
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
    } else {
      AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            m_isSimulation ? this::getSimulationSpeeds : this::getCurrentSpeeds, // Current ChassisSpeeds supplier
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
  }

  private void saveActivePath(List<Pose2d> activePath) {
    field.getObject("activePath").setPoses(activePath);
  }

  private void saveTargetPose(Pose2d target) {
    field.getObject("target").setPose(target);
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

  public void setPathPlannerSpeed(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    // wheelSpeeds.desaturate(0.5);

    m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, false);
  }

  public void setScale(double scale) {
    m_scale = scale;
  }

  // Get the pose of the robot as Pose2d
  public Pose2d getPose(){
    var pose = driveOdometry.getPoseMeters();
    return pose;
  }


  // Reset the Pose2d of the robot
  // This gets called if the path has an initial pose - which ours does.
  public void resetPose(Pose2d pose2d) {
    this.resetEncoders();
    this.driveOdometry.resetPosition(
        m_gyro.getRotation2d().unaryMinus(),
        getLDistance(),
        getRDistance(),
        pose2d);
    System.out.printf("Resetting pose to %s%n", pose2d.toString());
  }

  public ChassisSpeeds getCurrentSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftDriveEncoder.getVelocity(), m_rightDriveEncoder.getVelocity());
    var speeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    return speeds;
  }

  /**
   * This really should just use getCurrentSpeeds, but unfortunately the REVPhysicsSim doesn't seem to support
   * a way of setting the encoder velocity, so although the drivetrain simulator returns the simulated velocity,
   * we can't write those values to the encoders. As a result, this is a copy of getCurrentSpeeds with the only
   * difference being the source of velocity.
   */
  private ChassisSpeeds getSimulationSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(
      m_drivetrainSim.getLeftVelocityMetersPerSecond(),
      m_drivetrainSim.getRightVelocityMetersPerSecond());
    var speeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    return speeds;
  }
  
  public void drive(ChassisSpeeds speeds){
    setPathPlannerSpeed(speeds);
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since
   * last reset.
   *
   * @return Distance, in meters.
   */
  public double getLDistance() {
    return m_leftDriveEncoder.getPosition();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since
   * last reset.
   *
   * @return Distance in meters.
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
    driveOdometry.update(m_gyro.getRotation2d().unaryMinus(), getLDistance(), getRDistance());
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

  public void onSimulationInit() {
    m_drivetrainSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      Drive.WHEEL_GEAR_RATIO,
      // FIXME: These values are defaults from
      // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
      // and really should be measured.
      7.5,
      60.0,
      // This value is the wheel radius in metres
      Drive.WHEEL_CIRCUM / 100. / Math.PI / 2.,
      // FIXME: Turn this into a constant - it's used above as well.
      0.56,
      // TODO: Add noise to the simulation here as standard deviation values for noise:
      // x, y in m
      // heading in rad
      // l/r velocity m/s
      // l/r position in m
      VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
    );

    // Add all the motors for simulation. Unknown if the rear motor simulation is needed
    // or contributes anything.
    REVPhysicsSim.getInstance().addSparkMax(m_leftDriveMotorF, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_leftDriveMotorR, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_rightDriveMotorF, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_rightDriveMotorR, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This code is basically a duplicate of the getPeriod method in REVPhysicsSim.SimProfile.
    // It would be much better for debugging if we could simulate time passing rather than
    // run it in realtime, but unfortunately we have at least two different timers running
    // 1. Command scheduler timer
    // 2. REVPhysicsSim.SimProfile uses System.nanoTime
    // We want to ensure that our drivetrain simulator does its update with an equivalent
    // duration, so it's copied here for now.
    if (!m_startedSimulation) {
      m_lastSimTime = System.nanoTime();
      m_startedSimulation = true;
    }
    long now = System.nanoTime();
    final double period = (now - m_lastSimTime) / 1000000000.;
    m_lastSimTime = now;

    // Run the REV physics simulation so that the motor values change
    REVPhysicsSim.getInstance().run();

    // Provide the drivetrain simulation inputs from the motors.
    m_drivetrainSim.setInputs(m_leftDriveMotorF.get() * m_leftDriveMotorF.getBusVoltage(), m_rightDriveMotorF.get() * m_rightDriveMotorF.getBusVoltage());

    // Run the drivetrain simulation for the same amount of time as the physics simulation ran.
    m_drivetrainSim.update(period);

    // Update the position encoders. NOTE: It would be great to update velocity here but we can't do it
    // because the encoder we have has no method for updating velocity.
    m_leftDriveEncoder.setPosition(m_drivetrainSim.getLeftPositionMeters());
    m_rightDriveEncoder.setPosition(m_drivetrainSim.getRightPositionMeters());

    // Update the gyro to match the simulated heading
    // Suggested code from https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(m_drivetrainSim.getHeading().getDegrees());
  }
}
