package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Visionster extends SubsystemBase {
    // Create a reference to the table and entries
    private NetworkTable table;
    private NetworkTableEntry exampleEntry;
    double counter = 0;
    public boolean VisionCheck;

    public Visionster() {
        // Get the default instance of NetworkTables and the table "datatable"
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("datatable");

        // Create an entry in the table
        exampleEntry = table.getEntry("Astrobot");
        exampleEntry.setDouble(counter);

        // Optionally, print that the robot has initialized
        System.out.println("NetworkTables initialized and ready.");
    }

    @Override
    public void periodic() {
        // Example: setting a value

        // Example: getting a value and printing it
        double receivedValue = exampleEntry.getDouble(0); // Default to 0.0 if not set
        if (receivedValue > 0) {
            this.VisionCheck = true;
            exampleEntry.setDouble(0);
            System.out.println("Received value: " + receivedValue);
        }
        

        // You can add logic here to use the received value in your robot code
    }
}
