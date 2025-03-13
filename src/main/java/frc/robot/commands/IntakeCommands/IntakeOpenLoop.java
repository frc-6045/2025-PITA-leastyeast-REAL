package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

/** Either controller triggers move intake. */
public class IntakeOpenLoop extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final CommandXboxController controller;
    
    public IntakeOpenLoop(IntakeSubsystem intakeSubsystem, CommandXboxController xboxController) {
        m_IntakeSubsystem = intakeSubsystem;
        controller = xboxController;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute() {
        double triggerAxis = -controller.getLeftTriggerAxis()+controller.getRightTriggerAxis();
        m_IntakeSubsystem.setSpeed(triggerAxis); //Grant's Ternary IS GONE NOW :'(. Press right on stick and it won't run second rollers. Should be a button but I couldn't figure out how to return bool from a button if its pressed or not :)
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();   
    }

}
