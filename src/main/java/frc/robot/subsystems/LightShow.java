package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Vision;

public class LightShow extends SubsystemBase {

  private final int klightPWM = 9;

  // NOTE: you can only allocate ONE strip of LEDs (in series). This is a
  // limitation
  // of how WPILib implements the addressable LED strips.
  // https://www.chiefdelphi.com/t/can-not-allocate-a-second-addressableled-pwm-port/376859

  AddressableLED leds = new AddressableLED(klightPWM);
  final int ROWS = 8;
  final int COLS = 32;
  final int PIXELS = (ROWS * COLS);

  AddressableLEDBuffer pixels = new AddressableLEDBuffer(PIXELS);

  String m_lastBuffer = pixels.toString();

  Timer timer = new Timer();
  double waitTime = 0.0;

  public LightShow() {
    leds.setLength(PIXELS);
    leds.start();
    timer.start();
  }

  public void fillPercentage(int redPercent, int greenPercent, int bluePercent) {
    for (int i = 0; i < PIXELS; i++) {
      pixels.setRGB(i, redPercent, greenPercent, bluePercent);
    }
  }

  public void setBlue() {
    fillPercentage(0, 0, 10);
  }

  public void setPurple() {
    fillPercentage(5, 0, 5);
  }

  public void setGreen() {
    fillPercentage(0, 5, 0);
  }

  public void setRed() {
    fillPercentage(5, 0, 0);
  }

  // The lights sort of follow a nonlinear pattern where if x is even
  // convert x and y coordinates to an index on the light strip
  int findIndex(int x, int y) {
    int index;
    x = 31 - x;// correct from right to left
    if (x % 2 == 0) {
      index = (8 * x) + y;
    } else {
      index = (8 * x) + (7 - y);
    }
    return index;
  }

  @Override
  public void periodic() {

    // double currentTime = timer.get();//1.5034234234
    // double tenTimesFaster = currentTime * 10;//15.034234234
    // int index = (int) tenTimesFaster;//15

    // int x = 0;
    // int y = 2;
    // int index = findIndex(x, y);

    // pixels.setRGB(index % PIXELS, 20, 0, 0);
    // pixels.setRGB(findIndex(3, 3), 20, 0, 0);
    var currentBuffer = pixels.toString();
    var needsUpdate = m_lastBuffer != currentBuffer;
    if (needsUpdate) {
      m_lastBuffer = currentBuffer;
      leds.setData(pixels);
    }
  }

}
