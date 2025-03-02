// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.Constants.MotorConstants;

import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Driver controller D-Pad moves the climber. */
public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_ClimbMotor;
  private final boolean goUp;

  /**
   * Creates a new ClimberCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param up Direction the Climber should move
   */
  public ClimbCommand(ClimbSubsystem ClimbMotor, boolean up) {
    m_ClimbMotor = ClimbMotor;
    goUp = up; // (maybe wrong) true goes counterclockwise when facing robot from side with less clutter
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ClimbMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MotorConstants.kClimbMotorSpeed;
    if (goUp) m_ClimbMotor.setSpeed(speed);
    else m_ClimbMotor.setSpeed(-1*speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimbMotor.stopClimbMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
