package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import javax.net.ssl.TrustManagerFactorySpi;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.sim.DrivebaseSim;

public class DrivebaseSubsystem extends SubsystemBase {

  CANSparkMax m_leftDriveMotorF;
  CANSparkMax m_leftDriveMotorR;
  CANSparkMax m_rightDriveMotorF;
  CANSparkMax m_rightDriveMotorR;

  private final DifferentialDrive m_drive;
  DifferentialDriveKinematics m_kinematics;

  RelativeEncoder m_leftDriveEncoder;
  RelativeEncoder m_rightDriveEncoder;

  public PIDController m_leftPid;
  public PIDController m_rightPid;

  /*
   * LTVUnicycleController has default values copied here:
   * It provides a linear time-varying unicycle controller with default maximum
   * desired error tolerances of (0.0625 m, 0.125 m, 2 rad) and default maximum
   * desired control effort of (1 m/s, 2 rad/s).
   */
  private static final double DEFAULT_X_TOLERANCE_M = 0.0625;
  private static final double DEFAULT_Y_TOLERANCE_M = 0.125;
  private static final double DEFAULT_HEADING_TOLERANCE_RAD = 2.0;
  private static final double DEFAULT_HEADING_TOLERANCE_DEG = Math.toDegrees(DEFAULT_HEADING_TOLERANCE_RAD);
  private static final double DEFAULT_LINEAR_CONTROL_M_S = 1.0;
  private static final double DEFAULT_ANGULAR_CONTROL_RAD_S = 2.0;
  private static final double DEFAULT_ANGULAR_CONTROL_DEG_S = Math.toDegrees(DEFAULT_ANGULAR_CONTROL_RAD_S);
  private static final double DEFAULT_DT_S = 0.02;

  // LTV error tolerances are errors in x and y in meters, and in radians for
  // heading.
  private Vector<N3> m_ltvTolerance = VecBuilder.fill(DEFAULT_X_TOLERANCE_M, DEFAULT_Y_TOLERANCE_M,
      DEFAULT_HEADING_TOLERANCE_RAD);
  // LTV control effort values are in m/s and rad/s respectively.
  private Vector<N2> m_ltvControlEffort = VecBuilder.fill(DEFAULT_LINEAR_CONTROL_M_S, DEFAULT_ANGULAR_CONTROL_RAD_S);
  // LTV discretization step in seconds.
  private double m_ltvDt = DEFAULT_DT_S;

  SlewRateLimiter filter = new SlewRateLimiter(0);

  private double m_scale = 1;

  private DrivebaseSim m_drivebaseSim;

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

  // Put widgets meant for drivebase tuning here
  private ShuffleboardTab m_tuningTab = Shuffleboard.getTab("Drivebase Tuning");
  private GenericEntry m_ltvXTolerance;
  private GenericEntry m_ltvYTolerance;
  private GenericEntry m_ltvHeadingTolerance;
  private GenericEntry m_ltvLinearControl;
  private GenericEntry m_ltvAngularControl;
  private GenericEntry m_ltvDtEntry;
  private GenericEntry m_pidKpEntry;
  private GenericEntry m_pidKiEntry;
  private GenericEntry m_pidKdEntry;
  private GenericEntry m_usePidEntry;
  private GenericEntry m_syncedEntry;

  // Use PIDs in before calling tankDrive.
  private boolean m_experimentalPID = true;

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

    // REVERSE: m_leftDriveMotorF inverted. if we switch left to false and right to
    // true, it will be reversed
    m_leftDriveMotorR.follow(m_leftDriveMotorF);
    m_leftDriveMotorF.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);
    m_rightDriveMotorR.follow(m_rightDriveMotorF);
    m_rightDriveMotorF.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

    m_leftDriveMotorF.getPIDController().setP(0.2);
    m_rightDriveMotorF.getPIDController().setP(0.2);

    m_drive = new DifferentialDrive(m_leftDriveMotorF, m_rightDriveMotorF);
    // If we don't want to use motor.set, we can control it more like this:
    // m_drive = new DifferentialDrive(
    // (double value) -> m_leftDriveMotorF.getPIDController().setReference(value,
    // ControlType.kDutyCycle),
    // (double value) -> m_rightDriveMotorF.getPIDController().setReference(value,
    // ControlType.kDutyCycle)
    // );

    setDrivebaseIdle(IdleMode.kBrake);
    m_leftDriveEncoder = m_leftDriveMotorF.getEncoder();
    m_rightDriveEncoder = m_rightDriveMotorF.getEncoder();
    // when robot goes forward, left encoder spins positive and right encoder spins
    // negative

    // Position is measured in motor revolutions by default, but we want metres.
    // Wheel encoder scaling gives us centimetres and takes the gear ratio into
    // account,
    // and the scaling values appear to take more accurate wheel measurements and cm
    // -> m
    m_leftDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.LEFT_SCALING);
    m_rightDriveEncoder.setPositionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.RIGHT_SCALING);

    // Distance conversion is the same as the position factor, but convert from
    // minutes to seconds
    // since velocity is measured in rpm by default but we want m/s.
    m_leftDriveEncoder.setVelocityConversionFactor(m_leftDriveEncoder.getPositionConversionFactor() / 60);
    m_rightDriveEncoder.setVelocityConversionFactor(m_rightDriveEncoder.getPositionConversionFactor() / 60);

    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);

    // When we had no velocity, we liked kp=0.2
    m_leftPid = new PIDController(0.6, 0, 0);
    m_rightPid = new PIDController(0.6, 0, 0);

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
    m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH_METERS);
    driveOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLDistance(), getRDistance());

    PathPlannerLogging.setLogActivePathCallback(this::saveActivePath);
    PathPlannerLogging.setLogTargetPoseCallback(this::saveTargetPose);

    setupTuningTab();

    AutoBuilder.configureLTV(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        m_ltvTolerance,
        m_ltvControlEffort,
        m_ltvDt,
        new ReplanningConfig(),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
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

  private void setupTuningTab() {
    m_ltvXTolerance = m_tuningTab.add("x err (m)", DEFAULT_X_TOLERANCE_M)
        .withPosition(0, 0)
        .getEntry();
    m_ltvXTolerance.setDouble(m_ltvTolerance.get(0));

    m_ltvYTolerance = m_tuningTab.add("y err (m)", DEFAULT_Y_TOLERANCE_M)
        .withPosition(1, 0)
        .getEntry();
    m_ltvYTolerance.setDouble(m_ltvTolerance.get(1));

    m_ltvHeadingTolerance = m_tuningTab.add("angle err (deg)", DEFAULT_HEADING_TOLERANCE_DEG)
        .withPosition(2, 0)
        .getEntry();
    m_ltvHeadingTolerance.setDouble(Math.toDegrees(m_ltvTolerance.get(2)));

    m_ltvLinearControl = m_tuningTab.add("speed (m_s)", DEFAULT_LINEAR_CONTROL_M_S)
        .withPosition(0, 1)
        .getEntry();
    m_ltvLinearControl.setDouble(m_ltvControlEffort.get(0));

    m_ltvAngularControl = m_tuningTab
        .add("rotation (deg_s)", DEFAULT_ANGULAR_CONTROL_DEG_S)
        .withPosition(1, 1)
        .getEntry();
    m_ltvAngularControl.setDouble(Math.toDegrees(m_ltvControlEffort.get(1)));

    m_ltvDtEntry = m_tuningTab.add("dt (s)", DEFAULT_DT_S)
        .withPosition(0, 2)
        .getEntry();
    m_ltvDtEntry.setDouble(m_ltvDt);

    m_pidKpEntry = m_tuningTab.add("PID kP", 1.0)
        .withPosition(0, 3)
        .getEntry();
    m_pidKpEntry.setDouble(m_leftPid.getP());

    m_pidKiEntry = m_tuningTab.add("PID kI", 0.0)
        .withPosition(1, 3)
        .getEntry();
    m_pidKiEntry.setDouble(m_leftPid.getI());

    m_pidKdEntry = m_tuningTab.add("PID kD", 0.0)
        .withPosition(2, 3)
        .getEntry();
    m_pidKdEntry.setDouble(m_leftPid.getD());

    m_usePidEntry = m_tuningTab.add("Use PID", true)
        .withPosition(3, 3)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    m_usePidEntry.setBoolean(m_experimentalPID);

    m_syncedEntry = m_tuningTab.add("Synced", false).withPosition(1, 4).getEntry();
    m_syncedEntry.setBoolean(false);

    var syncCommand = new InstantCommand(() -> {
      if (!RobotState.isDisabled()) {
        System.out.println("Ignoring sync, please disable the robot first!");
        m_syncedEntry.setBoolean(false);
        return;
      }

      System.out.println("Synchronizing tuning parameters, please ignore error about reconfiguration.");
      double xTolerance = m_ltvXTolerance.getDouble(DEFAULT_X_TOLERANCE_M);
      double yTolerance = m_ltvYTolerance.getDouble(DEFAULT_Y_TOLERANCE_M);
      double headingToleranceDeg = m_ltvHeadingTolerance.getDouble(DEFAULT_HEADING_TOLERANCE_DEG);
      double linearControlEffort = m_ltvLinearControl.getDouble(DEFAULT_LINEAR_CONTROL_M_S);
      double angularControlDeg = m_ltvAngularControl.getDouble(DEFAULT_ANGULAR_CONTROL_DEG_S);
      double dt = m_ltvDtEntry.getDouble(DEFAULT_DT_S);

      AutoBuilder.configureLTV(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getCurrentSpeeds, // Current ChassisSpeeds supplier
          this::drive, // Method that will drive the robot given ChassisSpeeds
          VecBuilder.fill(xTolerance, yTolerance, Math.toRadians(headingToleranceDeg)),
          VecBuilder.fill(linearControlEffort, Math.toRadians(angularControlDeg)),
          dt,
          new ReplanningConfig(),
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
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

      double kP = m_pidKpEntry.getDouble(1.0);
      double kI = m_pidKiEntry.getDouble(0.0);
      double kD = m_pidKdEntry.getDouble(0.0);
      boolean usePid = m_usePidEntry.getBoolean(true);
      m_leftPid.setP(kP);
      m_rightPid.setP(kP);
      m_leftPid.setI(kI);
      m_rightPid.setI(kI);
      m_leftPid.setD(kD);
      m_rightPid.setD(kD);
      m_experimentalPID = usePid;
      if (m_experimentalPID) {
        System.out.printf("Using PID with values %f, %f, and %f%n", kP, kI, kD);
      } else {
        System.out.println("PID disabled");
      }
      m_syncedEntry.setBoolean(true);
    }, this).ignoringDisable(true).withName("Sync Now");
    m_tuningTab.add("Sync", syncCommand).withPosition(0, 4);
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

  public void disable() {
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
    final double maxSpeedMetersPerSecond = 4;
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(maxSpeedMetersPerSecond);

    if (m_experimentalPID) {
      // Weirdly there were times that it works better if we don't provide the
      // velocity?
      // Somehow the maxSpeedMetersPerSecond helps with that, because we are setting a
      // speed control in [-1,1] but we
      // get speeds from the path planner in meters per second and our simulation bot
      // can go a little above 5 m/s.
      // var leftOutput = m_leftPid.calculate(0, wheelSpeeds.leftMetersPerSecond);
      // var rightOutput = m_rightPid.calculate(0, wheelSpeeds.rightMetersPerSecond);

      var leftOutput = m_leftPid.calculate(m_leftDriveEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
      var rightOutput = m_rightPid.calculate(m_rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
      // System.out.printf("%f %f %f, %f %f %f%n", m_leftDriveEncoder.getVelocity(),
      // wheelSpeeds.leftMetersPerSecond, leftOutput,
      // m_rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond,
      // rightOutput);
      m_drive.tankDrive(leftOutput / maxSpeedMetersPerSecond, rightOutput / maxSpeedMetersPerSecond, false);
    } else {
      m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond / maxSpeedMetersPerSecond,
          wheelSpeeds.rightMetersPerSecond / maxSpeedMetersPerSecond, false);
    }

  }

  public void setScale(double scale) {
    m_scale = scale;
  }

  // Get the pose of the robot as Pose2d
  public Pose2d getPose() {
    var pose = driveOdometry.getPoseMeters();
    // System.out.println("The pose is: " + pose.toString());
    return pose;
  }

  // Reset the Pose2d of the robot
  // This gets called if the path has an initial pose - which ours does.
  public void resetPose(Pose2d pose2d) {
    // this.resetEncoders();
    this.driveOdometry.resetPosition(
        m_gyro.getRotation2d(),
        getLDistance(),
        getRDistance(),
        pose2d);
    System.out.printf("Resetting pose to %s%n", pose2d.toString());
  }

  public ChassisSpeeds getCurrentSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftDriveEncoder.getVelocity(),
        m_rightDriveEncoder.getVelocity());
    var speeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    return speeds;
  }

  public void drive(ChassisSpeeds speeds) {
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
    // retrying. We'll print the stats as we change states.
    for (int i = 0; i < 5; i++) {
      // Reset but check that return code is good. In a future world
      // we can do more like recording the specific error code, but
      // since we're adding this mid-competition I want to be conservative.
      boolean leftOk = m_leftDriveEncoder.setPosition(0) == REVLibError.kOk;
      boolean rightOk = m_rightDriveEncoder.setPosition(0) == REVLibError.kOk;

      if (leftOk && rightOk) {
        break; // Success! Get us out of here.
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
  public void periodic() {
    driveOdometry.update(m_gyro.getRotation2d(), getLDistance(), getRDistance());
    rotationScale = rotationScaleWidget.getDouble(0.76);
    leftDistanceWidget.setDouble(getLDistance());
    rightDistanceWidget.setDouble(getRDistance());

    // This publishes changes through to the dashboard.
    field.setRobotPose(driveOdometry.getPoseMeters());

    if (isBrakeMode != brakeModeWidget.getBoolean(false)) {
      isBrakeMode = brakeModeWidget.getBoolean(false);
      if (isBrakeMode) {
        setDrivebaseIdle(IdleMode.kBrake);
      }

      if (isCoastMode != coastModeWidget.getBoolean(false)) {
        isCoastMode = coastModeWidget.getBoolean(false);
        if (isCoastMode) {
          setDrivebaseIdle(IdleMode.kCoast);
        }
      }
    }
    navxMonitorWidget.setDouble(m_gyro.getAngle());

    counter++;

    if (counter == 50) {
      counter = 0;
    }
    counterWidget.setDouble(counter);
  }

  public void onSimulationInit() {
    m_drivebaseSim = new DrivebaseSim(m_leftDriveMotorF, m_rightDriveMotorF);
  }

  @Override
  public void simulationPeriodic() {
    m_drivebaseSim.periodic();
  }
}
