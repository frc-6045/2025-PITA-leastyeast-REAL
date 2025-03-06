package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Run intake for a set period of time either direction. */
public class IntakeClosedLoop extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    private final boolean direction;
    
    public IntakeClosedLoop(IntakeSubsystem intakeSubsystem, double time, boolean direction) {
        m_IntakeSubsystem = intakeSubsystem;
        this.time = time;
        addRequirements(m_IntakeSubsystem);
        this.direction=direction;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("intake!!!!!!!!!!!!!!!!!!!");
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            m_IntakeSubsystem.setSpeed(direction ? 1 : -1, direction ? 1 : -1);
            //System.out.println("timer value: "+ timer.get() + " time: " + time);
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
        m_IntakeSubsystem.stopIntake();  
        System.out.println("stop intake!!!!!!!!!!!!!!!!!!!");
    }

}
