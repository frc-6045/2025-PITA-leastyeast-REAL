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
    private final boolean has2;
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, 
                            ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        armSetpoint = armSetPoint;
        elevatorSetpoint = elevatorSetPoint;
        armSetpoint2 = .9;
        elevatorSetpoint2 = 1.434;
        has2=false;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
    }

    //constructor if the setpoint has 2 versions
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, double armSetPoint2, 
                        ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint, double elevatorSetPoint2) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        armSetpoint = armSetPoint;
        armSetpoint2 = armSetPoint2;
        elevatorSetpoint = elevatorSetPoint;
        elevatorSetpoint2 = elevatorSetPoint2;
        has2=true;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
    }
    
    @Override
    public void execute() {
        if (Bindings.isShift() && has2) {
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
