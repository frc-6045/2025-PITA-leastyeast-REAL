// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoring;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private Side side;
  private SwerveSubsystem swerveSubsystem;
  private Supplier<Translation2d> coralOffset;

  public AlignToReefTagRelative(Side side, SwerveSubsystem swerveSubsystem, Supplier<Translation2d> coralOffset) {
    xController = new PIDController(Constants.PositionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.PositionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.PositionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation

    this.side = side;
    this.swerveSubsystem = swerveSubsystem;
    this.coralOffset = coralOffset;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    int pipeline = 1;
    if (side == Side.RIGHT) {
      pipeline = 2;
    }

    LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT, pipeline);
  }
  
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(Constants.LIMELIGHT)) {
      double m_tx = LimelightHelpers.getTX(Constants.LIMELIGHT);
      //double m_coral_Offset = coralOffset.get().getY();
      
      double xSpeed = -xController.calculate(0);
      double ySpeed = -yController.calculate(m_tx);// - m_coral_Offset);
      double rotValue = -rotController.calculate(0);
      
      swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
    } else {
      swerveSubsystem.drive(new Translation2d(), 0, false);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(), 0, false);
    LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT, 0);
  }
  
  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return false;
  }
}