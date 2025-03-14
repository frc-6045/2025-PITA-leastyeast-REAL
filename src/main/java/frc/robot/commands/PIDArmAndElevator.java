package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Bindings;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.PositionConstants.Setpoints;
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
    private int encoderAt0 = 0;
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

    // I'M REALLY BORED DON'T JUDGE
    public PIDArmAndElevator(ArmSubsystem arm, ElevatorSubsystem elev, Setpoints setpoint) {
        m_ArmSubsystem = arm;
        m_ElevatorSubsystem = elev;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
        switch(setpoint) {
            case INTAKE:
                armSetpoint = PositionConstants.kHumanArmPosition;
                elevatorSetpoint = PositionConstants.kHumanElevatorPosition;
                armSetpoint2 = PositionConstants.kHumanGapArmPosition;
                elevatorSetpoint2 = PositionConstants.kHumanGapArmPosition;
                break;
            case L1:
                armSetpoint = armSetpoint2 = PositionConstants.kL1ArmPosition;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kL1ElevatorPosition;
                break;
            case L2:
                armSetpoint = armSetpoint2 = PositionConstants.kL2ArmPosition;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kL2ElevatorPosition;
                break;
            case L3:
                armSetpoint = PositionConstants.kL3ArmPosition;
                elevatorSetpoint = PositionConstants.kL3ElevatorPosition;
                armSetpoint2 = PositionConstants.kL3GapArmPosition;
                elevatorSetpoint2 = PositionConstants.kL3GapElevatorPosition;
                break;
            case L4:
                armSetpoint = PositionConstants.kL4ArmPosition;
                elevatorSetpoint = PositionConstants.kL4ElevatorPosition;
                armSetpoint2 = PositionConstants.kL4GapArmPosition;
                elevatorSetpoint2 = PositionConstants.kL4GapElevatorPosition;
                break;
            case ALGAE_HIGH:
                armSetpoint = armSetpoint2 = PositionConstants.kHighAlgaeArmPosition;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kHighAlgaeElevatorPosition;
                break;
            case ALGAE_LOW:
                armSetpoint = armSetpoint2 = PositionConstants.kLowAlgaeArmPosition;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kLowAlgaeElevatorPosition;
                break;
            case BARGE:
                armSetpoint = armSetpoint2 = PositionConstants.kBargeArm;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kBargeElev;
                break;
            default: //home
                armSetpoint = armSetpoint2 = PositionConstants.kHomeArmPosition;
                elevatorSetpoint = elevatorSetpoint2 = PositionConstants.kHomeElevatorPosition;
                break;
        }
    }
    
    @Override
    public void initialize() {
        if (Bindings.getOperatorShiftPressed()) has2=true;
        else has2=false;
    }
    
    @Override
    public void execute() {
        if (m_ArmSubsystem.getAbsoluteEncoderPosition()==0) {
            encoderAt0++;
        }
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
        if (encoderAt0>4) {
            System.out.println("encoder not plugged in?");
            return true;
        }
        if (m_ArmSubsystem.atSetpoint() && m_ElevatorSubsystem.atSetpoint()) {
            System.out.println("hello pidarmandelev is done :3");
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
