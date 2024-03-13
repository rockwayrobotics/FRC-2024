package frc.robot.commands.autoSequences;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotate;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.commands.ShootFromGroundDriveFour;
import frc.robot.commands.ShootFromGroundDriveRotateFour;
import frc.robot.commands.ShootSequenceFullAuto;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;


/** Four piece auto from the middle of the field, starting on the red alliance.
 * Different for each alliance so we don't smack into the truss first 
 */
public class middleFourPieceRed extends SequentialCommandGroup {
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

    // TODO optimize speeds and note positions 

    //  TODO why does it sometimes  break? weird behavioru lol 

    public middleFourPieceRed(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led, double drivemoreoffset){
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
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingMagenta)))
                
                .then(new InstantCommand(() -> m_intake.setBelt(0.8)))
                .then(new InstantCommand(() -> m_intake.setIntake(0.5)))
                .then(new DriveDistance(m_drivebase, -0.5, 1))

                .then(new ShootFromGroundDriveFour(m_shooter, m_intake, m_led, m_drivebase, 1))

                .then(new DriveDistance(m_drivebase, -0.5, 0.1))
                .then(new DriveRotate(m_drivebase, 25))

                .then(new InstantCommand(() -> m_intake.setBelt(0.8)))
                .then(new InstantCommand(() -> m_intake.setIntake(0.5)))
                .then(new DriveDistance(m_drivebase, -0.5, 1 + drivemoreoffset))

                .then(new ShootFromGroundDriveRotateFour(m_shooter, m_intake, m_led, m_drivebase, 1 - 0.1, -33))

                .then(new DriveDistance(m_drivebase, -0.5, 0.1))
                .then(new DriveRotate(m_drivebase, -25))

                .then(new DriveDistance(m_drivebase, -0.5, 1 + 0.04))
                .then(new ShootFromGroundDriveRotateFour(m_shooter, m_intake, m_led, m_drivebase, 1 - 0.1, 25))

                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kCoast)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

        this.addCommands(sequence);
    }
}