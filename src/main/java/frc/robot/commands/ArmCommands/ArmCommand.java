// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.Constants.MotorConstants;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmMotor;
  private final boolean goUp;
  private final CommandXboxController m_Controller;

  /**
   * Creates a new TestSparkFlex.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem armMotor, boolean up, CommandXboxController m_driverController) {
    m_ArmMotor = armMotor;
    goUp = up;
    m_Controller = m_driverController;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MotorConstants.kSparkFlexArmMotorSpeed;
    System.out.println(m_ArmMotor.getAbsoluteEncoderPosition());
    if (goUp) {
        m_ArmMotor.setSpeed(speed*m_Controller.getLeftTriggerAxis());
    } else {
        m_ArmMotor.setSpeed(-1*speed*m_Controller.getRightTriggerAxis());
    }
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
