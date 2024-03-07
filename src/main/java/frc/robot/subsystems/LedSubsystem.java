package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.modes;

import edu.wpi.first.wpilibj.Filesystem;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class LedSubsystem extends SubsystemBase{

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int counter = 0; 
  private int row = 0;
  
  private int m_pacer = 1;
  private modes m_mode;

  private int m_rainbowFirstPixelHue;

  BufferedImage image;

  public LedSubsystem(){
    m_led = new AddressableLED(Constants.LED.LED_PWM);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    m_mode = modes.Rainbow;
    
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private double sin_wave(double val, int set_point){
    return Math.sin(Math.PI * 2 * val / 70) * 70 + set_point;
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 0);
    }
  }

  private void orange() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 165, 0);
    }
  }

  private void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 128, 0);
    }
  }

  private void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 128);
    }
  }

  private void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 128, 0);
    }
  }

  private void purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 128);
    }
  }

  private void breathing_monochrome(int hue) {
    int sat = 0;
    for (int i=0; i < m_ledBuffer.getLength(); i++){
      sat = (int) sin_wave((double) i + counter, 200);
      m_ledBuffer.setHSV(i, hue, 255, sat);
    }
    counter++;
  }

  private void imageLoad(String imagePath) {
    try {
      image = ImageIO.read(new File(Filesystem.getDeployDirectory(), imagePath));
      m_mode = modes.imageLoop;
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private void imageLoop() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      int pixel = image.getRGB(i, row);
      int r = (pixel >> 16) & 0xff; 
      int g = (pixel >> 8) & 0xff;
      int b = (pixel >> 0) & 0xff; 
      m_ledBuffer.setRGB(i, r, g, b);
    }

    if (++row >= image.getHeight()){
      row = 0; 
    }
  }

  public void setMode(modes mode) {
    m_mode = mode;
    counter = 0;
    System.out.println("Set LED to: " + mode);
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    if (m_pacer == 3){
      switch(m_mode) {
          case Green:
              green();
              break;
          case Red:
              red();
              break;
          case Orange:
              orange();
              break;
          case Blue:
              blue();
              break;
          case Yellow:
              yellow();
              break;
          case Purple:
              purple();
              break;
          case BreathingYellow:
            breathing_monochrome(30);
            break;
          case BreathingMagenta:
            breathing_monochrome(150);
            break;
          case imageLoop:
            imageLoop();
            break;
          case badApple:
            imageLoad("images/badapple.png");
            break;
          case heatGradient:
            imageLoad("images/heatgradient.png");
            break;
          default:
              rainbow();
      }
    m_pacer = -1;
    }
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    m_pacer++;
  }
}

  