package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Subsystem for managing Limelight vision processing and AprilTag detection.
 * Provides robot pose estimation using AprilTag-based vision measurements.
 */
public class LimelightSubsystem extends SubsystemBase {
  private final String limelightName;
  private final SwerveSubsystem swerveSubsystem;
  private ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  // Store latest pose data
  private Pose2d robotPose = new Pose2d();
  private boolean hasTarget = false;
  private double lastUpdateTime = 0;

  /**
   * Creates a new LimelightSubsystem.
   *
   * @param swerveSubsystem The swerve drive subsystem to update with vision measurements
   */
  public LimelightSubsystem(SwerveSubsystem swerveSubsystem) {
    this.limelightName = Constants.LIMELIGHT;
    this.swerveSubsystem = swerveSubsystem;

    // Add Shuffleboard telemetry
    limelightTab.addBoolean("Has Target", this::hasTarget);
    limelightTab.addDouble("Robot X", () -> robotPose.getX());
    limelightTab.addDouble("Robot Y", () -> robotPose.getY());
    limelightTab.addDouble("Robot Theta (deg)", () -> robotPose.getRotation().getDegrees());
    limelightTab.addDouble("Last Update Time", () -> lastUpdateTime);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms by default)
    updateVisionMeasurement();
  }

  /**
   * Updates the robot's pose estimation using Limelight AprilTag detection.
   * Integrates vision measurements with the swerve drive's pose estimator.
   */
  private void updateVisionMeasurement() {
    // Get the latest pose estimate from Limelight using MegaTag2
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    // Check if we have valid AprilTag data
    if (poseEstimate != null && poseEstimate.tagCount > 0) {
      hasTarget = true;

      // Extract the pose from the estimate
      robotPose = poseEstimate.pose;

      // Get the timestamp of this measurement
      double timestamp = poseEstimate.timestampSeconds;
      lastUpdateTime = Timer.getFPGATimestamp();

      // Determine standard deviations based on distance and number of tags
      // More tags and closer distance = more confident measurement
      double xyStdDev = 0.5; // Default standard deviation in meters
      double thetaStdDev = 6.0; // Default standard deviation in degrees

      // Adjust confidence based on number of tags seen
      if (poseEstimate.tagCount >= 2) {
        xyStdDev = 0.3;
        thetaStdDev = 3.0;
      }

      // Adjust confidence based on average tag distance
      if (poseEstimate.avgTagDist < 2.0) {
        xyStdDev *= 0.5; // More confident when closer
        thetaStdDev *= 0.5;
      } else if (poseEstimate.avgTagDist > 4.0) {
        xyStdDev *= 2.0; // Less confident when farther
        thetaStdDev *= 2.0;
      }

      // Add the vision measurement to the swerve drive's pose estimator
      swerveSubsystem.getSwerveDrive().addVisionMeasurement(
          robotPose,
          timestamp
      );
    } else {
      hasTarget = false;
    }
  }

  /**
   * Gets the current estimated robot pose from the Limelight.
   *
   * @return The robot's pose (x, y, theta)
   */
  public Pose2d getRobotPose() {
    return robotPose;
  }

  /**
   * Gets the X position of the robot in meters.
   *
   * @return X position in meters
   */
  public double getX() {
    return robotPose.getX();
  }

  /**
   * Gets the Y position of the robot in meters.
   *
   * @return Y position in meters
   */
  public double getY() {
    return robotPose.getY();
  }

  /**
   * Gets the rotation angle (theta) of the robot.
   *
   * @return Rotation angle as Rotation2d
   */
  public Rotation2d getTheta() {
    return robotPose.getRotation();
  }

  /**
   * Gets the rotation angle (theta) of the robot in degrees.
   *
   * @return Rotation angle in degrees
   */
  public double getThetaDegrees() {
    return robotPose.getRotation().getDegrees();
  }

  /**
   * Checks if the Limelight currently sees any AprilTags.
   *
   * @return true if AprilTags are detected, false otherwise
   */
  public boolean hasTarget() {
    return hasTarget;
  }

  /**
   * Gets the number of AprilTags currently detected.
   *
   * @return Number of AprilTags in view
   */
  public int getTagCount() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    return (poseEstimate != null) ? poseEstimate.tagCount : 0;
  }

  /**
   * Gets the average distance to detected AprilTags.
   *
   * @return Average distance in meters, or 0 if no tags detected
   */
  public double getAverageTagDistance() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    return (poseEstimate != null) ? poseEstimate.avgTagDist : 0.0;
  }

  /**
   * Sets the Limelight's LED mode.
   *
   * @param mode LED mode (0=default, 1=off, 2=blink, 3=on)
   */
  public void setLEDMode(int mode) {
    LimelightHelpers.setLEDMode_ForceOff(limelightName);
  }

  /**
   * Sets the Limelight's pipeline.
   *
   * @param pipeline Pipeline index (0-9)
   */
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(limelightName, pipeline);
  }
}
