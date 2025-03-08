package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    LEDPattern yellow;
    LEDPattern green;
    public LEDs(LEDPattern yellow, LEDPattern green) {
        m_led = new AddressableLED(0);

        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
        
       
        yellow = LEDPattern.solid(Color.kYellow);
        green = LEDPattern.solid(Color.kGreen);
        LEDPattern red = LEDPattern.solid(Color.kRed);
        LEDPattern blue = LEDPattern.solid(Color.kBlue);
    }
}