package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Bindings;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Move arm and elevator to their respective setpoints. */
public class PIDArmAndElevator extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double armSetpoint;
    private final double elevatorSetpoint;
    private final double armSetpoint2;
    private final double elevatorSetpoint2;
    private boolean has2;
    /**
     * Constructor for 1 setpoint
     * @param armSubsystem
     * @param armSetPoint
     * @param elevatorSubsystem
     * @param elevatorSetPoint
     */
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, 
                            ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        armSetpoint = armSetPoint;
        elevatorSetpoint = elevatorSetPoint;
        armSetpoint2 = armSetPoint;
        elevatorSetpoint2 = armSetPoint;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
    }

    /**
     * Include second setpoint that will be run if shift button is pressed (we are running out of buttons)
     * @param armSubsystem
     * @param armSetPoint
     * @param armSetPoint2
     * @param elevatorSubsystem
     * @param elevatorSetPoint
     * @param elevatorSetPoint2
     */
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, double armSetPoint2, 
                        ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint, double elevatorSetPoint2) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        armSetpoint = armSetPoint;
        armSetpoint2 = armSetPoint2;
        elevatorSetpoint = elevatorSetPoint;
        elevatorSetpoint2 = elevatorSetPoint2;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
    }
    
    @Override
    public void initialize() {
        if (Bindings.isShift()) has2=true;
        else has2=false;
    }
    
    @Override
    public void execute() {
        if (has2) {
            m_ArmSubsystem.goToSetpoint(armSetpoint2);
            m_ElevatorSubsystem.goToSetpoint(elevatorSetpoint2);
        } else{
            m_ArmSubsystem.goToSetpoint(armSetpoint);
            m_ElevatorSubsystem.goToSetpoint(elevatorSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_ArmSubsystem.atSetpoint() && m_ElevatorSubsystem.atSetpoint()) {
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
