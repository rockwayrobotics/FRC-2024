package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Visionster;

public class DriveDistanceVision extends Command {

  private DrivebaseSubsystem m_drivebase;
  private double m_speed;
  private Supplier<Double> m_distanceSupplier;
  private double m_distance;
  private Visionster m_vision;
  private double m_leftBase;
  private double m_rightBase;
  private double m_leftDist;
  private double m_rightDist;

  public DriveDistanceVision(DrivebaseSubsystem subsystem, double speed, Supplier<Double> distance, Visionster vision) {

    m_drivebase = subsystem;
    m_speed = speed;
    m_distanceSupplier = distance;
    m_vision = vision;

    addRequirements(m_drivebase, m_vision);
  }

  public DriveDistanceVision(DrivebaseSubsystem subsystem, double speed, double distance, Visionster vision) {
    this(subsystem, speed, () -> { return distance; }, vision);
  }

  @Override
  public void initialize() {
    m_distance = m_distanceSupplier.get();

    // Resets encoder values to default
    // System.out.println("Current Encoders: " + m_drivebase.getRDistance());
    // m_drivebase.resetEncoders();

    // System.out.println("After Encoders: " + m_drivebase.getRDistance());
    m_leftBase = m_drivebase.getLDistance();
    m_rightBase = m_drivebase.getRDistance();

    System.out.printf("Moving: %.3f from %.3f, %.3f%n",
      m_distance, m_leftBase, m_rightBase);
  }

  @Override
  public void execute() {
    if (m_vision.VisionCheck){
      m_drivebase.set(0,0);
    }
    else {
      m_drivebase.set(m_speed, 0);
    }
    // System.out.println("Executing");
  }

  @Override
  public boolean isFinished() {
    // System.out.println("Current pos: " + Math.abs(m_drivebase.getRDistance()) + " Setpoint: " + m_distance);
    //SmartDashboard.putNumber("Auto Command Distance Travelled", m_drivebase.getRDistance());
    double m_leftDist = m_drivebase.getLDistance() - m_leftBase;
    double m_rightDist = m_drivebase.getRDistance() - m_rightBase;
    double distance = (m_leftDist + m_rightDist) / 2.0;
    return (Math.abs(distance) >= m_distance);
  }

  @Override
  public void end(boolean cancelled) {
    m_drivebase.stop(); // Resets the drivebase to 0, ends command
    System.out.printf("Moved: %.3f, %.3f%n", m_leftDist, m_rightDist);
  }
}
