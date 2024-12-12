package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestLogCommand extends SequentialCommandGroup {

public TestLogCommand() {
  this.addCommands(new InstantCommand(() -> System.out.println("Test Log Command")));
  this.addCommands(new WaitCommand(2));
  this.addCommands(new InstantCommand(() -> System.out.println("it worked")));
  }
}