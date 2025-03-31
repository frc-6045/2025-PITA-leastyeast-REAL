/*
 * The LEDs are 12 volt.
 * 
 * They need to be blue normally, but if Adam auto-aligns and the robot can see an AprilTag, they should be green.
 */


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private static final int kPort = 0;
    private static final int kLength = 120;  

    private AddressableLED m_LED;
    private AddressableLEDBuffer m_LEDBuffer;
    
    public LedSubsystem() {
      System.out.println("led subsystem init");
      m_LED = new AddressableLED(kPort);
      m_LEDBuffer = new AddressableLEDBuffer(kLength);
      m_LED.setLength(kLength);
      //m_LED.setData(m_LEDBuffer);
      m_LED.start();

      setDefaultCommand(runPattern(LEDPattern.solid(Color.kAquamarine)).withName("Blue"));
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      m_LED.setData(m_LEDBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
      System.out.println("hi");
      return run(() -> pattern.applyTo(m_LEDBuffer));

       
    }
  }