package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run the intake at a velocity set by an Elastic dashboard slider.
 * Useful for testing and tuning intake speeds for prototypes.
 */
public class IntakeVelocityFromDashboard extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private static final String SLIDER_KEY = "INTAKE Test RPM";

    /**
     * Creates a new IntakeVelocityFromDashboard command.
     * The target RPM is read from SmartDashboard using the key "INTAKE Test RPM".
     * @param intakeSubsystem The intake subsystem
     */
    public IntakeVelocityFromDashboard(IntakeSubsystem intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        // Initialize the slider with a default value if it doesn't exist
        if (!SmartDashboard.containsKey(SLIDER_KEY)) {
            SmartDashboard.putNumber(SLIDER_KEY, 0);
        }
    }

    @Override
    public void execute() {
        // Read the target RPM from the dashboard and apply it
        double targetRPM = SmartDashboard.getNumber(SLIDER_KEY, 0);
        m_IntakeSubsystem.setVelocityRPM(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
