package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

/** Either controller triggers move intake. */
public class IntakeIntake extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final CommandXboxController controller;
    private final BooleanSupplier coralDetected;
    private final Timer timer = new Timer();
    private final double speed;
    boolean coralExist = false;
    
    public IntakeIntake(IntakeSubsystem intakeSubsystem, CommandXboxController xboxController, BooleanSupplier coralDetected, double speed) {
        m_IntakeSubsystem = intakeSubsystem;
        controller = xboxController;
        this.coralDetected = coralDetected;
        this.speed = speed;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        coralExist = coralDetected.getAsBoolean();
    }

    @Override
    public void execute() {
        double triggerAxis = -controller.getLeftTriggerAxis()+controller.getRightTriggerAxis();
        m_IntakeSubsystem.setSpeed(triggerAxis*speed);
        if (coralDetected.getAsBoolean() && timer.get() == 0 && !coralExist) {
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 0.01) {
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();   
    }

}
