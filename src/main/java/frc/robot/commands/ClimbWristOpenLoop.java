package frc.robot.commands;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbWristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbWristOpenLoop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbWristSubsystem m_Wrist;
  private final double speed;

  public ClimbWristOpenLoop(ClimbWristSubsystem ClimbMotor, double speed) {
    m_Wrist = ClimbMotor;
    this.speed = speed;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.setSpeed(speed);
    m_Wrist.setSpeed(-1*speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.stopClimbMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
