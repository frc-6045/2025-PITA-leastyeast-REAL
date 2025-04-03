package frc.robot.commands.AutoScoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
  private PIDController yController, rotController;
  private Side side;
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier coralOffset;
  private CommandXboxController driverController;
  private int pipeline;

  public AlignToReefTagRelative(Side side, SwerveSubsystem swerveSubsystem, CommandXboxController driverController, DoubleSupplier coralOffset) {
    yController = new PIDController(Constants.PositionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Strafe left/right
    rotController = new PIDController(Constants.PositionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation

    this.side = side;
    this.swerveSubsystem = swerveSubsystem;
    this.driverController = driverController;
    this.coralOffset = coralOffset;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    pipeline = 1;
    if (side == Side.RIGHT) {
      pipeline = 2;
    }

    LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT, pipeline);
  }
  
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(Constants.LIMELIGHT)) {
      double driver_speed = driverController.getLeftY();
      double m_tx = LimelightHelpers.getTX(Constants.LIMELIGHT);
      double m_coralOffset = coralOffset.getAsDouble();

      yController.setTolerance(0.00);
      
      double ySpeed = -yController.calculate(m_tx - m_coralOffset);
      double rotValue = driverController.getRightX();
      
      swerveSubsystem.drive(new Translation2d(driver_speed * 2, ySpeed), rotValue, false);
    } else {
      System.out.println("Limelight Cannot See April Tag");

      swerveSubsystem.drive(new Translation2d(), 0, false);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT, 0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}