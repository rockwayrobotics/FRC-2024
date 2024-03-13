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
import frc.robot.commands.ShootFromGroundDriveFour;
import frc.robot.commands.ShootFromGroundDriveRotate;
import frc.robot.commands.ShootFromGroundDriveRotateFour;
import frc.robot.commands.ShootSequenceFullAuto;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;


/** Autonomously shoots the piece loaded in auto, and drives outside the short side of the community
 * <p><strong>SETUP:</strong> Place the robot on short side of community beside charge station
 * <p><strong>END:</strong> The robot has driven beyond the line to get out of the community, but not too far beyond the line
 * <p><strong>SCORES:</strong> Piece in auto, Auto mobility
 */
public class middleFourPiece extends SequentialCommandGroup {
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

    public middleFourPiece(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led, double waittime, double drivedistance, double drivemoreoffset){
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_intake = intake;
        m_led = led; 

        m_drivebase.setDrivebaseIdle(IdleMode.kBrake);

        addRequirements(m_drivebase, m_shooter, m_intake);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Red)))
                .then(new InstantCommand(() -> m_intake.setBelt(-0.5)))
                .then(new WaitCommand(0.2))
                .then(new InstantCommand(() -> m_intake.setBelt(0)))
                .then(new ShootSequenceFullAuto(m_shooter, m_intake, m_led))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Blue)))
                .then(new WaitCommand(waittime))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.BreathingMagenta)))
                .thenWithTimeout(new DriveDistance(m_drivebase, -1, drivedistance), 5)
                .then(new ShootFromGroundDriveFour(m_shooter, m_intake, m_led, m_drivebase, drivedistance))
                .then(new DriveDistance(m_drivebase, -0.5, 0.1))
                .then(new DriveRotate(m_drivebase, 30))
                .thenWithTimeout(new DriveDistance(m_drivebase, -0.7, drivedistance + drivemoreoffset), 5)
                .then(new ShootFromGroundDriveRotateFour(m_shooter, m_intake, m_led, m_drivebase, drivedistance, -30 + drivemoreoffset))
                .then(new DriveDistance(m_drivebase, -0.5, 0.1))
                .then(new DriveRotate(m_drivebase, -30))
                .thenWithTimeout(new DriveDistance(m_drivebase, -0.7, drivedistance + drivemoreoffset), 5)
                .then(new ShootFromGroundDriveRotateFour(m_shooter, m_intake, m_led, m_drivebase, drivedistance, 30 + drivemoreoffset))
                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kCoast)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));

        this.addCommands(sequence);
    }
}