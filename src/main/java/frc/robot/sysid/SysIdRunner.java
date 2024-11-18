package frc.robot.sysid;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.DrivebaseSubsystem;

public class SysIdRunner {
  private SysIdRoutine m_routine;
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.zero(Units.Volts);
  private final MutableMeasure<Distance> m_distance = MutableMeasure.zero(Units.Meters);
  private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.zero(Units.MetersPerSecond);

  public SysIdRunner(DrivebaseSubsystem drivebase) {
    // Default config is 1 volt/sec ramp rate and 7 volt step voltage.
    var config = new SysIdRoutine.Config();
    var mechanism = new SysIdRoutine.Mechanism(voltage -> {
      drivebase.sysIdDrive(voltage);
    }, log -> {
      drivebase.sysIdUpdateMeasures(false, m_appliedVoltage, m_distance, m_velocity);
      log.motor("drive-left")
          .voltage(m_appliedVoltage)
          .linearPosition(m_distance)
          .linearVelocity(m_velocity);
      drivebase.sysIdUpdateMeasures(true, m_appliedVoltage, m_distance, m_velocity);
      log.motor("drive-right")
          .voltage(m_appliedVoltage)
          .linearPosition(m_distance)
          .linearVelocity(m_velocity);
    }, drivebase);
    m_routine = new SysIdRoutine(config, mechanism);
  }

  public Command quasistatic(SysIdRoutine.Direction direction) {
    return m_routine.quasistatic(direction);
  }

  public Command dynamic(SysIdRoutine.Direction direction) {
    return m_routine.dynamic(direction);
  }

  public Command sequence() {
    return new SequentialCommandGroup(
        this.quasistatic(Direction.kForward),
        this.quasistatic(Direction.kReverse),
        this.dynamic(Direction.kForward),
        this.dynamic(Direction.kReverse));
  }
}
