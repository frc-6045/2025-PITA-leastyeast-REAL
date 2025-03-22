package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

/** Either controller triggers move intake. */
public class IntakeOpenLoop extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final CommandXboxController controller;
    private final double speed;
    
    public IntakeOpenLoop(IntakeSubsystem intakeSubsystem, CommandXboxController xboxController, double speed) {
        m_IntakeSubsystem = intakeSubsystem;
        controller = xboxController;
        this.speed = speed;
        
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute() {
        double triggerAxis = -controller.getLeftTriggerAxis() + controller.getRightTriggerAxis();
        
        m_IntakeSubsystem.setSpeed(triggerAxis * speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();   
    }
}
