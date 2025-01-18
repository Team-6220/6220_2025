package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE = null;

    // Might need to change these values
    private static final int kPort = 9;
    private static final int kLength = 120;
 
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;
 
    public LEDSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      m_led.setLength(kLength);
      m_led.start();
 
      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      // Other default patterns could be used instead
      setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }


    // Set the LED strip to red
    public void setRed() {
        // Create an LED pattern that sets the entire strip to solid red
        LEDPattern red = LEDPattern.solid(Color.kRed);

        // Apply the LED pattern to the data buffer
        red.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    // Set the LED strip to green
    public void setGreen() {
        // Create an LED pattern that sets the entire strip to solid green
        LEDPattern green = LEDPattern.solid(Color.kGreen);

        // Apply the LED pattern to the data buffer
        green.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    // Set the LED strip to gold
    public void setGold() {
        // Create an LED pattern that sets the entire strip to solid gold
        LEDPattern gold = LEDPattern.solid(Color.kGold);

        // Apply the LED pattern to the data buffer
        gold.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    // Set the LED strip to rainbow
    public void setRainbow() {
        // Create an LED pattern that sets the entire strip to a rainbow pattern
        LEDPattern rainbow = LEDPattern.rainbow(255, 0);

        // Apply the LED pattern to the data buffer
        rainbow.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    //Set the LED strip to half 1 color, half another color
    public void setHalfHalf() {
        // Create an LED pattern that displays the first half of a strip as solid red,
        // and the second half of the strip as solid blue.
        // These colors can be changed
        // Changing the number will change where they split
        LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));

        // Apply the LED pattern to the data buffer
        steps.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    public void setBreathe() {
        // Create an LED pattern that displays a red solid color, breathing at a 2 second period (0.5 Hz)
        LEDPattern base = LEDPattern.solid(Color.kRed);
        LEDPattern pattern = base.breathe(Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern.applyTo(m_buffer);

        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }


    @Override
    public void periodic() {
      // Periodically send the latest LED color data to the LED strip for it to display
      m_led.setData(m_buffer);
    }


    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return run(() -> pattern.applyTo(m_buffer));
    }

    public static synchronized LEDSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LEDSubsystem();
        }
        return INSTANCE;
  }
}