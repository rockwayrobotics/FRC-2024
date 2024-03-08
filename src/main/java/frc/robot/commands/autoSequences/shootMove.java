package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.commands.ShootSequenceFull;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Map;


/** Autonomously shoots the piece loaded in auto, and drives outside the short side of the community
 * <p><strong>SETUP:</strong> Place the robot on short side of community beside charge station
 * <p><strong>END:</strong> The robot has driven beyond the line to get out of the community, but not too far beyond the line
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility
 */
public class shootMove extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;
    LedSubsystem m_led;

    public void setStatusWidget(SimpleWidget AutoFailedWidget, FailFastTimeoutGroup sequence) {
        if(sequence.timedOut()) {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "red"));
        } else {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "green"));
        }
    }

    public shootMove(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, IntakeSubsystem intake){
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_intake = intake;

        addRequirements(m_drivebase, m_shooter, m_intake);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new DriveDistance(drivebase, 0.5, 2))
                .then(new ShootSequenceFull(m_shooter, m_intake))
                .then(new WaitCommand(0.7))
                .thenWithTimeout(new DriveDistance(drivebase, -0.5, 80), 10);


        this.addCommands(sequence);
    }
}