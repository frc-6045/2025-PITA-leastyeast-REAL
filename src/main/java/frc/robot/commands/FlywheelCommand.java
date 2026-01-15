package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Runs the flywheel at the target RPM set on the dashboard.
 * Designed to be used with whileTrue() - runs while button is held.
 */
public class FlywheelCommand extends Command {
    private final FlywheelSubsystem m_Flywheel;

    public FlywheelCommand(FlywheelSubsystem flywheel) {
        m_Flywheel = flywheel;
        addRequirements(m_Flywheel);
    }

    @Override
    public void initialize() {
        // Reset PID when command starts (optional, can help with integral windup)
    }

    @Override
    public void execute() {
        // Get target RPM from dashboard and run flywheel
        double targetRPM = m_Flywheel.getTargetRPMFromDashboard();
        m_Flywheel.runToRPM(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_Flywheel.stop();
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own - runs while button is held
        return false;
    }
}
