package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDCANdle extends SubsystemBase {
private static LEDCANdle INSTANCE = null;
  private CANdle candleLED;
  private final CANdleConfiguration CANdleConfiguration;
  private int stripLength=308;
  private double smootherStripLength=308.0;


  
// Initialize the LEDCANdle's configuration
  public LEDCANdle() {
    // Creates a new CANdle with ID 0
    candleLED = new CANdle(1);
    CANdleConfiguration = new CANdleConfiguration();
    CANdleConfiguration.stripType = LEDStripType.RGB;
    CANdleConfiguration.statusLedOffWhenActive = true;
    // Makes the CANdle not disable when it loses communication to the controller
    CANdleConfiguration.disableWhenLOS = false;
    // Can be set to 0.5 for half brightness, 1 for full brightness, etc.
    CANdleConfiguration.brightnessScalar = 1.0;
    CANdleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    // Configures all persistent settings
    candleLED.configAllSettings(CANdleConfiguration, 100);
    candleLED.configBrightnessScalar(0.5);

  }

  // Use RGB values to set the color of all the LEDs
  public void setColor(int r, int g, int b, int w, int stId, int count, boolean isBlinking) {
    candleLED.clearAnimation(0);
    candleLED.setLEDs(0, 0, 0, 0, 0, 308);
    candleLED.setLEDs(r, g, b, w, stId, count);
    // candleLED.animate(new RgbFadeAnimation(b, 0.5, count));
    // Modulates the VBat output to the specified duty cycle percentage
    candleLED.modulateVBatOutput(0.95);
    if (isBlinking) {
        

        //while on, wait .5 sec, turn off
        //while off, wait .5 sec, turn on
    }
  }

    /** 
     * @param brightness
     * 1 is max brightness
     * @param speed
     * 0.5 is half speed
     * @param LEDs
     * 8 is the number of LEDs (Might need to change)
    */
  public void setRainbow() {
    // candleLED.animate(new RainbowAnimation(.5, 0.5, 308));
    candleLED.animate(new FireAnimation(.025, .02, 308, 1,0.001, false, 8), 0);
  }

  public void setRai(boolean up){
    if(up&&stripLength<308){
      stripLength++;
    }
    else if(!up&&stripLength>0){
      stripLength--;
    }
    candleLED.setLEDs(0, 0, 0, 0, stripLength-1, 308);
    //System.out.println(stripLength);
    candleLED.animate(new RainbowAnimation(1.0, 5.0, stripLength), 0);
  }
  public void setTesting(boolean up){
    if(up&&smootherStripLength<=308){
      smootherStripLength+=0.6;
    }
    else if(!up&&smootherStripLength>=0){
      smootherStripLength-=0.5;
    }
    smootherStripLength=Math.min(Math.max(0, smootherStripLength), 308);
    System.out.println(smootherStripLength);
    candleLED.setLEDs(0, 0, 0, 0, (int)Math.round(smootherStripLength-1), 308);
    candleLED.setLEDs(0, 245, 15, 5, 0, (int)Math.round(smootherStripLength));
  }
  public void turnOff() {
    candleLED.setLEDs(0, 0, 0);
    candleLED.modulateVBatOutput(0);
    candleLED.clearAnimation(0);
  }

  public void setLarson(int r, int g, int b) {
    candleLED.animate(new LarsonAnimation(r, g, b));
  }

  public static synchronized LEDCANdle getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new LEDCANdle();
    }
    return INSTANCE;
}
}

