package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;

/**
 * Subsystem for managing Limelight vision processing and AprilTag detection.
 * Provides robot pose estimation using AprilTag-based vision measurements.
 * Also supports neural network detection for Algae and Coral game pieces.
 */
public class LimelightSubsystem extends SubsystemBase {
  private final String limelightName;
  private final SwerveSubsystem swerveSubsystem;
  private ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  // Store latest pose data
  private Pose2d robotPose = new Pose2d();
  private boolean hasTarget = false;
  private double lastUpdateTime = 0;

  // Store latest neural detector data for Algae
  private boolean hasAlgae = false;
  private int algaeCount = 0;
  private double algaeTX = 0.0;
  private double algaeTY = 0.0;
  private RawDetection[] rawDetections = new RawDetection[0];
  private String detectorClassName = "";
  private boolean neuralDetectorEnabled = false;

  /**
   * Creates a new LimelightSubsystem.
   *
   * @param swerveSubsystem The swerve drive subsystem to update with vision measurements
   */
  public LimelightSubsystem(SwerveSubsystem swerveSubsystem) {
    this.limelightName = Constants.LIMELIGHT;
    this.swerveSubsystem = swerveSubsystem;

    // Add Shuffleboard telemetry for AprilTag tracking
    limelightTab.addBoolean("Has Target", this::hasTarget);
    limelightTab.addDouble("Robot X", () -> robotPose.getX());
    limelightTab.addDouble("Robot Y", () -> robotPose.getY());
    limelightTab.addDouble("Robot Theta (deg)", () -> robotPose.getRotation().getDegrees());
    limelightTab.addDouble("Last Update Time", () -> lastUpdateTime);

    // Add Shuffleboard telemetry for Neural Detector (Algae detection)
    limelightTab.addBoolean("Neural Detector Enabled", this::isNeuralDetectorEnabled);
    limelightTab.addBoolean("Has Algae", this::hasAlgae);
    limelightTab.addNumber("Algae Count", this::getAlgaeCount);
    limelightTab.addDouble("Algae TX", this::getAlgaeTX);
    limelightTab.addDouble("Algae TY", this::getAlgaeTY);
    limelightTab.addString("Detector Class", this::getDetectorClassName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms by default)
    updateVisionMeasurement();

    // Update neural detector results if enabled
    if (neuralDetectorEnabled) {
      updateNeuralDetection();
    }
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

  // ==================== NEURAL DETECTOR METHODS ====================

  /**
   * Enables the neural detector pipeline for Algae/Coral detection.
   * Switches to the neural detector pipeline and enables periodic updates.
   */
  public void enableNeuralDetector() {
    LimelightHelpers.setPipelineIndex(limelightName, VisionConstants.NEURAL_DETECTOR_PIPELINE);
    neuralDetectorEnabled = true;
  }

  /**
   * Disables the neural detector and switches back to the AprilTag pipeline.
   */
  public void disableNeuralDetector() {
    LimelightHelpers.setPipelineIndex(limelightName, VisionConstants.APRILTAG_PIPELINE);
    neuralDetectorEnabled = false;
    hasAlgae = false;
    algaeCount = 0;
  }

  /**
   * Updates neural detection results from the Limelight.
   * Called automatically in periodic() when neural detector is enabled.
   */
  private void updateNeuralDetection() {
    // Get raw detections from neural detector
    rawDetections = LimelightHelpers.getRawDetections(limelightName);

    // Get the primary detector class name
    detectorClassName = LimelightHelpers.getDetectorClass(limelightName);

    // Count Algae detections and find primary target
    algaeCount = 0;
    boolean foundPrimaryAlgae = false;

    for (RawDetection detection : rawDetections) {
      if (detection.classId == VisionConstants.ALGAE_CLASS_ID) {
        algaeCount++;

        // Use the first Algae detection as the primary target
        if (!foundPrimaryAlgae) {
          algaeTX = detection.txnc;
          algaeTY = detection.tync;
          foundPrimaryAlgae = true;
        }
      }
    }

    hasAlgae = algaeCount > 0;

    // Reset values if no Algae found
    if (!hasAlgae) {
      algaeTX = 0.0;
      algaeTY = 0.0;
    }
  }

  /**
   * Checks if the Limelight currently detects any Algae game pieces.
   *
   * @return true if Algae is detected, false otherwise
   */
  public boolean hasAlgae() {
    return hasAlgae;
  }

  /**
   * Gets the number of Algae game pieces currently detected.
   *
   * @return Number of Algae detected
   */
  public int getAlgaeCount() {
    return algaeCount;
  }

  /**
   * Gets the horizontal offset from the principal point to the primary Algae target.
   *
   * @return Horizontal offset in degrees (txnc)
   */
  public double getAlgaeTX() {
    return algaeTX;
  }

  /**
   * Gets the vertical offset from the principal point to the primary Algae target.
   *
   * @return Vertical offset in degrees (tync)
   */
  public double getAlgaeTY() {
    return algaeTY;
  }

  /**
   * Gets all raw neural detector results.
   *
   * @return Array of RawDetection objects for all detected game pieces
   */
  public RawDetection[] getRawDetections() {
    return rawDetections;
  }

  /**
   * Gets the primary Algae detection, or null if no Algae is detected.
   *
   * @return Primary Algae RawDetection object, or null
   */
  public RawDetection getPrimaryAlgaeDetection() {
    for (RawDetection detection : rawDetections) {
      if (detection.classId == VisionConstants.ALGAE_CLASS_ID) {
        return detection;
      }
    }
    return null;
  }

  /**
   * Gets the current detector class name from the neural detector.
   *
   * @return Class name string (e.g., "algae", "coral")
   */
  public String getDetectorClassName() {
    return detectorClassName;
  }

  /**
   * Checks if the neural detector is currently enabled.
   *
   * @return true if neural detector is enabled, false otherwise
   */
  public boolean isNeuralDetectorEnabled() {
    return neuralDetectorEnabled;
  }
}
