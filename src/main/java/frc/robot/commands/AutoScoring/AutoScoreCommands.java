// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoring;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCommands {

  private final SwerveSubsystem m_DriveSubsystem;
  private final ArmSubsystem m_ArmSubsystem; 
  private final ElevatorSubsystem m_ElevatorSubsystem; 
  private final IntakeSubsystem m_IntakeSubsystem;

  private final Timer timer = new Timer();
  
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);


  public AutoScoreCommands(
      SwerveSubsystem m_DriveSubsystem, 
      ArmSubsystem m_ArmSubsystem, ElevatorSubsystem m_ElevatorSubsystem, 
      IntakeSubsystem m_IntakeSubsystem) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_ElevatorSubsystem = m_ElevatorSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;

    timer.stop();
    timer.reset();
  }


  public Command scoreNearestReefFace(Setpoints setPoint, Supplier<Side> side, Supplier<Translation2d> offset) {
    if (side.get() != null) {

      Command commandToRun;

      Command MoveArmAndElevator;

      try {
        MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setPoint);
      } catch(Exception e) {
        MoveArmAndElevator = new PrintCommand("(i don't think this will ever be printed?) uhhh invalid setpoint " + setPoint);
      }
      
      commandToRun = m_DriveSubsystem.driveToFirstAutoScorePose(side.get()).andThen( 
      new ParallelDeadlineGroup(
        m_DriveSubsystem.driveToSecondAutoScorePose(side.get(), offset.get()), 
        MoveArmAndElevator),
      new IntakeClosedLoop(m_IntakeSubsystem, 0.8, true)
      );

      System.out.println("should be running");
      // commandToRun.addRequirements(m_ArmSubsystem, m_DriveSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
      return commandToRun;
    }
    else {
      System.out.println("hey you did not pick a side :(");
    }
    return Commands.print("huh what happened");
  }


  public Supplier<Command> scoreNearestReefFaceOther(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset, Supplier<Pose2d> closestTag) {
    return () -> {
    Pose2d firstPoseDriveTo, secondPoseDriveTo;
    Pose2d closestAprilTagPose = closestTag.get();

    // straight behind the april tag by quite a bit
    firstPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(-1,0));

    // left or right side of reef
    double sideOffset = (side.get() == Side.RIGHT) ? -0.2 : 0.2;
    secondPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(0.6, sideOffset));

    // intake offset
    secondPoseDriveTo = applyOffsetToPose(secondPoseDriveTo, offset.get());

    System.out.println("align to nearest reef face, side is " + side.get() +
      "\napriltag pose is " + closestAprilTagPose.getX() + " " + closestAprilTagPose.getY() + " " + closestAprilTagPose.getRotation() +
      "\ncoral offset is " + offset.get().getX() + " " + offset.get().getY());

    return
      new SequentialCommandGroup(
        m_DriveSubsystem.driveToPose(firstPoseDriveTo),
        new ParallelDeadlineGroup(
          m_DriveSubsystem.driveToPoseSlowMode(secondPoseDriveTo),
          new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setpoint)));
    };
  }


  public Command scoreSpecificReefPole(Setpoints setPoint, Pose2d pole, Supplier<Translation2d> offset) {
    return m_DriveSubsystem.driveToFirstAutoScorePose(pole).andThen( 
      new ParallelDeadlineGroup(
        m_DriveSubsystem.driveToSecondAutoScorePose(pole, offset.get()), 
        new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setPoint)),
      new IntakeClosedLoop(m_IntakeSubsystem, 0.8, true)
      );
    //return Commands.print("hi");
  }


  public Pose2d applyOffsetToPose(Pose2d pose, Translation2d offset) {
    Translation2d rotatedOffset = offset.rotateBy(pose.getRotation());
    return new Pose2d(pose.getTranslation().plus(rotatedOffset), pose.getRotation());
  }

  public Pose2d getAprilTagPose2d(int id) {
      var tagPose = aprilTagFieldLayout.getTagPose(id).get();
      return new Pose2d(tagPose.getX(), tagPose.getY(), new Rotation2d(tagPose.getRotation().getZ()));
    }

  public Pose2d closestAprilTag(Pose2d robotPose) {
    timer.start();
    // Use the robot pose and return the closest AprilTag on a REEF
    //List<Integer> tagIDs = List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11);
    List<Integer> tagIDs = List.of(17, 18, 19, 20, 21, 22);

    double minDistance = Double.MAX_VALUE;
    Pose2d closestTagPose = new Pose2d();
    Translation2d poseTranslation = robotPose.getTranslation();

    for (int tagID : tagIDs) {
      var tagPoseOptional = aprilTagFieldLayout.getTagPose(tagID);
      var tagPose = tagPoseOptional.get();
      Pose2d tagPose2d = new Pose2d(tagPose.getX(), tagPose.getY(), new Rotation2d(tagPose.getRotation().getZ()));
      double distance = poseTranslation.getDistance(tagPose2d.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestTagPose = tagPose2d;
      }
    }
    timer.stop();
    System.out.println("timer: " + timer.get());
    timer.reset();

    return closestTagPose;
  }


}
