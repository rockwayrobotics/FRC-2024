package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.modes;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private modes m_mode;
  private modes m_previous_mode;

  // private int counter;
  private boolean st = true;

  public LedSubsystem() {
    m_led = new AddressableLED(Constants.LED.LED_PWM);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_previous_mode = m_mode = modes.Green;
    m_led.start();
    System.err.println("led subsystem REWALLSA!!");
  }

  public void setMode(modes mode) {
    m_mode = mode;
    System.out.println("set mode called");
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

  private void one_spaced() {
    var i = 0;
    while (i < m_ledBuffer.getLength()) {
      if (i % 2 == 0) {
        m_ledBuffer.setRGB(i, 0, 0, 128);
        i += 1;
      } else {
        m_ledBuffer.setRGB(i, 0, 128, 0);
        i += 1;
      }
    }
  }

  // private int counta = 0;

  private void single_red_dot() {
    // for (int x = 0; x < m_ledBuffer.getLength(); x++) {
    // double r = 0;
    // double g = 0;
    // double b = 0;

    // if (x % 10 == counta % 10) {
    // g = 255;
    // }

    // m_ledBuffer.setRGB(x, (int) r, (int) g, (int) b);
    // }

    // counta = (counta + 1) % m_ledBuffer.getLength();
    for (int i = 1; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 128, 0);
      // m_ledBuffer.setRGB(i-1, 0, 0, 0);
    }
  }

  @Override
  public void periodic() {
    if (m_mode != m_previous_mode) {
      switch (m_mode) {
        case Green:
          green();
          break;
        case Blue:
          blue();
          break;
        case oneSpace:
          one_spaced();
          break;
        case singleRedDot:
          single_red_dot();
          break;
      }
      m_led.setData(m_ledBuffer);
    }
    m_previous_mode = m_mode;
    // Written by enoch (I take the blame)
    if (m_mode == modes.oneSpace) {
      for (int i = 0; i < m_ledBuffer.getLength(); i = i + 1) {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      if (st) {
        for (int i = 0; i < m_ledBuffer.getLength(); i = i + 2) {
          m_ledBuffer.setRGB(i, 128, 128, 0);
        }
        st = false;
      } else {
        for (int i = 1; i < m_ledBuffer.getLength(); i = i + 2) {
          m_ledBuffer.setRGB(i, 128, 128, 0);
        }
        st = true;
      }
    }
  }
}
