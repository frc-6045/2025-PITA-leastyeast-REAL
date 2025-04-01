/*
 * The LEDs are 12 volt.
 * 
 * They need to be blue normally, but if Adam auto-aligns and the robot can see an AprilTag, they should be green.
 * 
 * -0.49 always
 * -0.07 other times
 * 0.75 times that is not either of the other two
 */


 package frc.robot.subsystems;

 import edu.wpi.first.wpilibj.AddressableLED;
 import edu.wpi.first.wpilibj.AddressableLEDBuffer;
 import edu.wpi.first.wpilibj.LEDPattern;
 import edu.wpi.first.wpilibj.motorcontrol.Spark;
 import edu.wpi.first.wpilibj.util.Color;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class LedSubsystem extends SubsystemBase {
     private AddressableLEDBuffer m_LEDBuffer;
 
     private Spark m_Blinkin;
     
     public LedSubsystem() {
       System.out.println("led subsystem init");
       m_Blinkin = new Spark(9);
 
       m_Blinkin.set(0.81);
     }
     
     @Override
     public void periodic() {
 
     }
 
     public void set(double pattern) {
       m_Blinkin.set(pattern);
     }
   }