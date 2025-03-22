package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Run intake for a set period of time either direction. */
public class IntakeConditional extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final BooleanSupplier run;
    private final boolean direction;
    private final Timer timer = new Timer();
    private final double time;
    
    public IntakeConditional(IntakeSubsystem intakeSubsystem, BooleanSupplier run, boolean direction, double time) {
        m_IntakeSubsystem = intakeSubsystem;
        this.run = run;
        this.direction = direction;
        this.time = time;
        
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (run.getAsBoolean()) {
            m_IntakeSubsystem.setSpeed(direction ? 1 : -1);
        }
        else 
        {
            m_IntakeSubsystem.stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > time) {
            return true;
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();  
    }
}
