package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotate;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.commands.ShootFromGroundDrive;
import frc.robot.commands.ShootFromGroundDriveRotate;
import frc.robot.commands.ShootSequenceFullAuto;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;



/** Three piece auto from the middle of the field, starting on the blue alliance.
 * Different for each alliance so we don't smack into the truss first 
 */
public class middleThreePieceBlue extends SequentialCommandGroup {
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

    public middleThreePieceBlue(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led, double waittime, double drivedistance){
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_intake = intake;
        m_led = led; 

        m_drivebase.setDrivebaseIdle(IdleMode.kBrake);

        addRequirements(m_drivebase, m_shooter, m_intake);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Red)))
                .then(new InstantCommand(() -> m_intake.setBelt(-0.7)))
                .then(new WaitCommand(0.2))
                .then(new InstantCommand(() -> m_intake.setBelt(0)))
                .then(new ShootSequenceFullAuto(m_shooter, m_intake, m_led))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Blue)))
                .then(new WaitCommand(waittime))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingMagenta)))
                .thenWithTimeout(new DriveDistance(m_drivebase, -0.3, drivedistance), 5)
                .then(new ShootFromGroundDrive(m_shooter, m_intake, m_led, m_drivebase, drivedistance))
                .then(new DriveDistance(m_drivebase, -0.3, 0.1))
                .then(new DriveRotate(m_drivebase, -30))
                .thenWithTimeout(new DriveDistance(m_drivebase, -0.3, drivedistance + 0.15), 5)
                .then(new ShootFromGroundDriveRotate(m_shooter, m_intake, m_led, m_drivebase, drivedistance, 30 + 0.15))
                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kCoast)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

        this.addCommands(sequence);
    }
}