// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem swerveSubsystem;
  private boolean isRightScore;

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem swerveSubsystem) {
    xController = new PIDController(Constants.PositionConstants.X_REEF_ALIGNMENT_P, 0, 0);  // Vertical movement
    yController = new PIDController(Constants.PositionConstants.Y_REEF_ALIGNMENT_P, 0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.PositionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
  
    this.isRightScore = isRightScore;
    this.swerveSubsystem = swerveSubsystem;
    
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.PositionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.PositionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.PositionConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.PositionConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.PositionConstants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.PositionConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.PositionConstants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Limelight Target Count", LimelightHelpers.getTargetCount(""));

    if (LimelightHelpers.getTV("")) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      swerveSubsystem.drive(new Translation2d(yController.getError() < Constants.PositionConstants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      swerveSubsystem.drive(new Translation2d(), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.PositionConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.PositionConstants.POSE_VALIDATION_TIME);
  }
}