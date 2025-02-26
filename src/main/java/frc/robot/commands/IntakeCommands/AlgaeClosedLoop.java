package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.AlgaeRemovingSubsystem;

/** Run algae intake for a set period of time either direction. */
public class AlgaeClosedLoop extends Command {
    private final AlgaeRemovingSubsystem m_Algae;
    private final double time;
    private final Timer timer = new Timer();
    private final boolean direction;
    
    public AlgaeClosedLoop(AlgaeRemovingSubsystem algae, double time, boolean direction) {
        m_Algae = algae;
        this.time = time;
        addRequirements(m_Algae);
        this.direction=direction;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            m_Algae.setSpeed(MotorConstants.kAlgaeRemovingMotorSpeed * (direction ? 1 : -1));
            System.out.println("timer value: "+ timer.get() + " time: " + time);
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.get()>time) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Algae.stopMotor();  
    }

}
