package frc.robot.commands.AutoScoring;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoScoreUtil {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final List<Pose2d> tagPoses = getListOfApriltags();
    public static final DistanceUnit Meters = BaseUnits.DistanceUnit;
    public static final TimeUnit Seconds = BaseUnits.TimeUnit;
    public static final TimeUnit Second = Seconds;
    public static final LinearVelocityUnit MetersPerSecond = Meters.per(Second);
    public static final PathConstraints kPathConstraints = new PathConstraints(2, 1.75, 1/2 * Math.PI, 1 * Math.PI);

    // thanks Spartronics
    public static Command getPathFromWaypoint(Pose2d waypoint, SwerveSubsystem m_DriveSubsystem) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(m_DriveSubsystem.getPose().getTranslation(), getPathVelocityHeading(m_DriveSubsystem.getFieldVelocity(), waypoint, m_DriveSubsystem)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return 
                Commands.sequence(
                Commands.print("start position PID loop"),
                PositionPIDCommand.generateCommand(m_DriveSubsystem, waypoint, 0.5),
                Commands.print("end position PID loop")
            );
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            kPathConstraints,
            new IdealStartingState(getVelocityMagnitude(m_DriveSubsystem.getFieldVelocity()), m_DriveSubsystem.getHeading()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(m_DriveSubsystem, waypoint, 0.5),
            Commands.print("end position PID loop")
        );
    }

    // thanks Spartronics
    private static Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target, SwerveSubsystem m_DriveSubsystem){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(m_DriveSubsystem.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    // thanks Spartronics
    private static LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }
    


    public static Pose2d applyOffsetToPose(Pose2d pose, Translation2d offset) {
        Translation2d rotatedOffset = offset.rotateBy(pose.getRotation());
        return new Pose2d(pose.getTranslation().plus(rotatedOffset), pose.getRotation());
    }

    public static Pose2d getAprilTagPose2d(int id) {
        var tagPose = aprilTagFieldLayout.getTagPose(id).get();
        return new Pose2d(tagPose.getX(), tagPose.getY(), new Rotation2d(tagPose.getRotation().getZ()));
    }

    public static List<Pose2d> getListOfApriltags() {
      List<Pose2d> tagPoses = new ArrayList<>();
      for (int tagID : List.of(6,7,8,9,10,11, 17, 18, 19, 20, 21, 22)) {
        var tagPose = aprilTagFieldLayout.getTagPose(tagID).get();
        tagPoses.add(
          new Pose2d(tagPose.getX(), tagPose.getY(), new Rotation2d(tagPose.getRotation().getZ()))
        );
      }
      return tagPoses;
    }

    public static Pose2d closestAprilTag(Pose2d robotPose) {
        double minDistance = Double.MAX_VALUE;
        Pose2d closestTagPose = new Pose2d();
        Translation2d poseTranslation = robotPose.getTranslation();

        for (Pose2d tagPose : tagPoses) {
            double distance = poseTranslation.getDistance(tagPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestTagPose = tagPose;
            }
        }
        return closestTagPose;
    }
}
