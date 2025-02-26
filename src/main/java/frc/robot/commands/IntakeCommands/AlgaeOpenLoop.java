package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.AlgaeRemovingSubsystem;

public class AlgaeOpenLoop extends Command {
    private final AlgaeRemovingSubsystem m_Algae;
    private final boolean direction;
    public AlgaeOpenLoop(AlgaeRemovingSubsystem m_Algae, boolean direction) {
        this.m_Algae = m_Algae;
        this.direction = direction;
        addRequirements(m_Algae);
    }
    
    @Override
    public void execute() {
        System.out.println(direction ? MotorConstants.kAlgaeRemovingMotorSpeed : -MotorConstants.kAlgaeRemovingMotorSpeed);
        m_Algae.setSpeed(direction ? MotorConstants.kAlgaeRemovingMotorSpeed : -MotorConstants.kAlgaeRemovingMotorSpeed);
    }

    @Override
    public void end(boolean Interrupted) {
        m_Algae.stopMotor();
    }
}
