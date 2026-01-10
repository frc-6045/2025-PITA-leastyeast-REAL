package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Command to run the flywheel at the target RPM set via SmartDashboard.
 * Designed to be used with whileTrue() binding - runs while button is held.
 */
public class FlywheelCommand extends Command {
    private final FlywheelSubsystem m_Flywheel;

    /**
     * Creates a new FlywheelCommand.
     *
     * @param flywheel The flywheel subsystem to control
     */
    public FlywheelCommand(FlywheelSubsystem flywheel) {
        m_Flywheel = flywheel;
        addRequirements(m_Flywheel);
    }

    @Override
    public void execute() {
        // Run flywheel to target RPM (reads from SmartDashboard)
        m_Flywheel.runToTargetRPM();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop flywheel when button is released
        m_Flywheel.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own - controlled by button hold
        return false;
    }
}
