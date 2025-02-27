// WIP
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.AlgaeRemovingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Move arm and elevator to their respective setpoints. */
public class AlgaeShoot extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final AlgaeRemovingSubsystem m_AlgaeSubsystem;
    private final double armSetpoint;
    private final double elevatorSetpoint;
    private Timer timer;
    public AlgaeShoot(ArmSubsystem armSubsystem, 
                            ElevatorSubsystem elevatorSubsystem, AlgaeRemovingSubsystem algae) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        m_AlgaeSubsystem = algae;
        armSetpoint = PositionConstants.kAlgaeShootArm;
        elevatorSetpoint = PositionConstants.kAlgaeShootElev;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem, m_AlgaeSubsystem);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_ElevatorSubsystem.goToSetpoint(elevatorSetpoint);
        m_ArmSubsystem.goToSetpoint(armSetpoint);

    }

    @Override
    public boolean isFinished() {
        if (m_ArmSubsystem.atSetpoint() && m_ElevatorSubsystem.atSetpoint()) {
            // if (m_ArmSubsystem.atSetpoint()) {timer.start();
            // System.out.println("timer start: " + timer.get());}
            // if (timer.get()>6) {
            //     return true;
            //     //System.out.println("return true owoowowowo");
            // }
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean Interrupted) {
        //m_ArmSubsystem.stopArmMotor();
        m_ElevatorSubsystem.stopElevatorMotors();
    }

}
