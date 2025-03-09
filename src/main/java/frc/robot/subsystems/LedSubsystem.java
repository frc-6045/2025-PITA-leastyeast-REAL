package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    /** Creates a new LEDs. */
    private AddressableLED m_LED;
    private AddressableLEDBuffer m_LEDBuffer;
    
    public LedSubsystem() {
      m_LED = new AddressableLED(0);
      
      m_LEDBuffer = new AddressableLEDBuffer(60);
      m_LED.setLength(m_LEDBuffer.getLength());
     
      m_LED.setData(m_LEDBuffer);
      m_LED.start();
      
      for(int i = 0; i < m_LEDBuffer.getLength(); i++){
        m_LEDBuffer.setRGB(i, 39, 2, 201);
      }
  
      m_LED.setData(m_LEDBuffer);
    }
  
    public void setColor(int red, int green, int blue){
      for (int i = 0; i < m_LEDBuffer.getLength(); i++){
        m_LEDBuffer.setRGB(i, red, green, blue);
      }
      
      m_LED.setData(m_LEDBuffer);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }