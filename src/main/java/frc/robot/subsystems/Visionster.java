package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Visionster extends TimedRobot {
    // Create a reference to the table and entries
    private NetworkTable table;
    private NetworkTableEntry exampleEntry;

    @Override
    public void robotInit() {
        // Get the default instance of NetworkTables and the table "datatable"
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("datatable");

        // Create an entry in the table
        exampleEntry = table.getEntry("exampleValue");

        // Optionally, print that the robot has initialized
        System.out.println("NetworkTables initialized and ready.");
    }

    @Override
    public void teleopPeriodic() {
        // Example: setting a value
        exampleEntry.setDouble(123.45);

        // Example: getting a value and printing it
        double receivedValue = exampleEntry.getDouble(0.0); // Default to 0.0 if not set
        System.out.println("Received value: " + receivedValue);

        // You can add logic here to use the received value in your robot code
    }
}
