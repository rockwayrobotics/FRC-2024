package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;


/** Autonomously shoots the piece loaded in auto, and drives outside the short side of the community
 * <p><strong>SETUP:</strong> Place the robot on short side of community beside charge station
 * <p><strong>END:</strong> The robot has driven beyond the line to get out of the community, but not too far beyond the line
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility
 */
public class moveNoShoot extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    LedSubsystem m_led;

    public void setStatusWidget(SimpleWidget AutoFailedWidget, FailFastTimeoutGroup sequence) {
        if(sequence.timedOut()) {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "red"));
        } else {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "green"));
        }
    }

    public moveNoShoot(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led, double waittime, double drivedistance){
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_led = led; 

        m_drivebase.setDrivebaseIdle(IdleMode.kBrake);

        addRequirements(m_drivebase, m_shooter);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Blue)))
                .then(new WaitCommand(waittime))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingMagenta)))
                .thenWithTimeout(new DriveDistance(drivebase, -0.3, drivedistance), 5)
                .then(new WaitCommand(1))
                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kCoast)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

        this.addCommands(sequence);
    }
}