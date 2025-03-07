// NOT USED
package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ArmSubsystem;

/** Closed loop command that moves the arm back and forth once for coral L2. */
public class ArmFlick extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private double initialPosition;
    // state is 0 when going from initial position to setpoint 1
    // 1 when going from setpoint 1 to setpoint 2
    // 2 when complete
    private int state;

    public ArmFlick(ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = m_ArmSubsystem.getAbsoluteEncoderPosition();
        state=0;
        System.out.println("aaaaaaa");
  }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        System.out.println("desired pos: " + (initialPosition+0.05) + 
                            "\ncurrent pos: " + m_ArmSubsystem.getAbsoluteEncoderPosition() +
                            "\n" + m_ArmSubsystem.atSetpoint());
        if (state==0) {
            m_ArmSubsystem.goToSetpoint(initialPosition+PositionConstants.kArmFlickDistance1);
            if (m_ArmSubsystem.atSetpoint()) state=1;
        }
        else if (state==1) {
            m_ArmSubsystem.goToSetpoint(initialPosition+PositionConstants.kArmFlickDistance2);
            if (m_ArmSubsystem.atSetpoint()) state=2;
        }

    }

    @Override
    public boolean isFinished() {
        if (state==2) return true;
        return false;
    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
    }
}