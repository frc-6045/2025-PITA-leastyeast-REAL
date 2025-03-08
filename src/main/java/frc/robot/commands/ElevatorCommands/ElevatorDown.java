package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** Run intake for a set period of time either direction. */
public class ElevatorDown extends Command {
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    
    public ElevatorDown(ElevatorSubsystem elevator, double time) {
        m_ElevatorSubsystem = elevator;
        this.time = time;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("auto pulse work!");
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            m_ElevatorSubsystem.setSpeed(-MotorConstants.kElevatorMotorsSpeed);
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
        m_ElevatorSubsystem.stopElevatorMotors();  
    }

}
