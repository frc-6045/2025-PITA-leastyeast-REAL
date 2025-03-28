package frc.robot.commands;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbClosedLoop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_ClimbMotor;
  private final double m_Setpoint;
  private final double m_tolerance;

  public ClimbClosedLoop(ClimbSubsystem ClimbMotor, double setpoint, double tolerance) {
    m_ClimbMotor = ClimbMotor;
    m_Setpoint = setpoint;
    m_tolerance = tolerance;
    
    addRequirements(m_ClimbMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbMotor.setSpeed(-1);
  }

  // Called once the command ends or) is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimbMotor.stopClimbMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Setpoint+m_tolerance>m_ClimbMotor.getAbsoluteEncoderPosition() && m_ClimbMotor.getAbsoluteEncoderPosition()>m_Setpoint-m_tolerance) return true;
    return false;
  }
}
