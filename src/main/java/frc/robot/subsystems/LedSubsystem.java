package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private static final int kPort = 0;
    private static final int kLength = 200;  

    private AddressableLED m_LED;
    private AddressableLEDBuffer m_LEDBuffer;
    
    public LedSubsystem() {
      m_LED = new AddressableLED(kPort);
      m_LEDBuffer = new AddressableLEDBuffer(kLength);
      m_LED.setLength(m_LEDBuffer.getLength());
      m_LED.start();

      setDefaultCommand(runPattern(LEDPattern.solid(new Color(39, 2, 201))));
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      m_LED.setData(m_LEDBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
      return run(() -> pattern.applyTo(m_LEDBuffer));
    }
  }