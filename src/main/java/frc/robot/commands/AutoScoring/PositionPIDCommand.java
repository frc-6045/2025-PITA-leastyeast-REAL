package frc.robot.commands.AutoScoring;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = SwerveConstants.DRIVE_CONTROLLER;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final Timer timer = new Timer();

    // private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    // private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    // private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();

    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.0);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);



    private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

            boolean rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            boolean position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

            ChassisSpeeds speeds = mSwerve.getRobotVelocity();
            double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            boolean speed = linearSpeed < kSpeedTolerance.in(MetersPerSecond);

            // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(0.1);
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, double timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.lock();
        });
    }

    @Override
    public void initialize() {
        // endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        // endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.drive(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );

        // xErrLogger.accept(mSwerve.getPose().getX() - goalPose.getX());
        // yErrLogger.accept(mSwerve.getPose().getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        // endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.stop();

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + mSwerve.getRobotVelocity() + "m/s"
        );
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
