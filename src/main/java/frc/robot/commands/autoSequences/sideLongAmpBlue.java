package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoShootReset;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotate;
import frc.robot.commands.DriveUntilLoaded;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.commands.ShootFromGroundDriveRotateAdjustDriveAuto;
import frc.robot.commands.ShootSequenceFullAuto;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;



/** Two piece auto from the side of the field, starting on the red alliance.
 * Goes far out to the note on the source side
 */
public class sideLongAmpBlue extends SequentialCommandGroup {
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

    public sideLongAmpBlue(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led, double waittime){
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_intake = intake;
        m_led = led;

        // addRequirements(m_drivebase, m_shooter, m_intake);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new AutoShootReset(m_drivebase, m_intake, m_led))
                .then(new ShootSequenceFullAuto(m_shooter, m_intake, m_led))

                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Blue)))
                .then(new WaitCommand(waittime))

                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingMagenta)))

                .then(new DriveDistance(m_drivebase, -0.5, 0.5))
                .then(new DriveRotate(m_drivebase, 28))

                .then(new InstantCommand(() -> m_intake.setBelt(0.8)))
                .then(new InstantCommand(() -> m_intake.setIntake(0.5)))
                .then(new DriveUntilLoaded(m_drivebase, m_intake, -0.5, 4.35))

                .then(new ShootFromGroundDriveRotateAdjustDriveAuto(m_shooter, m_intake, m_led, m_drivebase, -45, 0.5, 1))
                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kCoast)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

        this.addCommands(sequence);
    }
}