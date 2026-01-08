package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Command to run the intake at a specific RPM using PID control. */
public class IntakeVelocityPID extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double targetRPM;

    /**
     * Creates a new IntakeVelocityPID command.
     * @param intakeSubsystem The intake subsystem
     * @param rpm Target velocity in RPM (positive = intake, negative = outtake)
     */
    public IntakeVelocityPID(IntakeSubsystem intakeSubsystem, double rpm) {
        m_IntakeSubsystem = intakeSubsystem;
        this.targetRPM = rpm;

        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.setVelocityRPM(targetRPM);
    }

    @Override
    public void execute() {
        // PID controller continuously updates to maintain target velocity
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
