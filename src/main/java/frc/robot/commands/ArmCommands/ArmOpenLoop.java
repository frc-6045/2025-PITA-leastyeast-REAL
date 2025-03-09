// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.Constants.MotorConstants;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Driver controller bumpers move arm. */
public class ArmOpenLoop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmMotor;
  private final boolean goUp;

  /**
   * Creates a new ArmOpenLoop.
   *
   * @param subsystem The subsystem used by this command.
   * @param up Direction the arm should move
   */
  public ArmOpenLoop(ArmSubsystem armMotor, boolean up) {
    m_ArmMotor = armMotor;
    goUp = up; // (maybe wrong) true goes counterclockwise when facing robot from side with less clutter
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MotorConstants.kArmMotorSpeed;
    //double triggerAxis = m_Controller.getLeftTriggerAxis()-m_Controller.getRightTriggerAxis();
    //speed*=triggerAxis;
    //System.out.println("open loop ARM: speed is " +speed + "\nencoder position is" + m_ArmMotor.getAbsoluteEncoderPosition());
    if (goUp) m_ArmMotor.setSpeed(speed);
    else m_ArmMotor.setSpeed(-1*speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmMotor.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
