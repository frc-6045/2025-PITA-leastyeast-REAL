package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** Operator D-pad up and down moves elevator. */
public class ElevatorOpenLoop extends Command { 
    private final ElevatorSubsystem m_elevatorSubsystem;
    private boolean goUp;
    public ElevatorOpenLoop(ElevatorSubsystem elevatorSubsystem, boolean up) {
    m_elevatorSubsystem = elevatorSubsystem;
    goUp = up;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MotorConstants.kElevatorMotorsSpeed;
    if (goUp) {
        m_elevatorSubsystem.setSpeed(speed);
    } else {
        m_elevatorSubsystem.setSpeed(-1*speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
