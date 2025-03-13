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
    
    public IntakeConditional(IntakeSubsystem intakeSubsystem, BooleanSupplier run, boolean direction) {
        m_IntakeSubsystem = intakeSubsystem;
        this.run = run;
        this.direction = direction;
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
            System.out.println("run!!!!");
            //System.out.println("timer value: "+ timer.get() + " time: " + time);
        }
        else {
            m_IntakeSubsystem.stopIntake();
            System.out.println("no run!!!!");
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 2) {
            System.out.println("KDJHFOISEDHTIEKSJNFKUDFJSMHGBKJSR");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();  
        System.out.println("IntakeConditional end");
    }

}
