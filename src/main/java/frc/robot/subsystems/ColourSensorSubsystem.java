// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColourSensorSubsystem extends SubsystemBase {
  private final ColorSensorV3 m_colourSensor;

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  
  private final ColorMatch m_colourMatcher = new ColorMatch();

  ShuffleboardTab colours = Shuffleboard.getTab("Colours");
  
    GenericEntry redValue =
      colours.add("Amount Of Red", 0)
         .getEntry();
         
    GenericEntry greenValue =
      colours.add("Amount Of Green", 0)
         .getEntry();
    
    GenericEntry blueValue =
      colours.add("Amount Of Blue", 0)
         .getEntry();
    
    GenericEntry IRValue = 
      colours.add("IR Value", 0)
      .getEntry();

    GenericEntry proximityValue =
      colours.add("Proximity Value", 0)
      .getEntry();

    GenericEntry colourDetected = 
      colours.add("Colour","nah man")
      .getEntry();


  /** Creates a new ColourSensorSubsystem. */
  public ColourSensorSubsystem() {
    m_colourSensor = new ColorSensorV3(Constants.I2C.COLOUR_SENSOR);

    m_colourMatcher.addColorMatch(kBlueTarget);
    m_colourMatcher.addColorMatch(kGreenTarget);
    m_colourMatcher.addColorMatch(kRedTarget);
    m_colourMatcher.addColorMatch(kYellowTarget);
  }

  // This method will be called once per scheduler run
  public void periodic() {

    Color detectedColour = m_colourSensor.getColor();
    double IR = m_colourSensor.getIR();
    int proximity = m_colourSensor.getProximity();

    redValue.setDouble(detectedColour.red);
    greenValue.setDouble(detectedColour.green);
    blueValue.setDouble(detectedColour.blue);
    IRValue.setDouble(IR);
    proximityValue.setInteger(proximity);

    String colourString;

    ColorMatchResult match = m_colourMatcher.matchClosestColor(detectedColour);

    if (match.color == kBlueTarget) {
      colourString = "Blue";
    } else if (match.color == kRedTarget) {
      colourString = "Red";
    } else if (match.color == kGreenTarget) {
      colourString = "Green";
    } else if (match.color == kYellowTarget) {
      colourString = "Yellow";
    } else {
      colourString = "NUH UH";
    }

    colourDetected.setString(colourString);
  }
}
