// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoring;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import static frc.robot.commands.AutoScoring.AutoScoreUtil.closestAprilTag;
import static frc.robot.commands.AutoScoring.AutoScoreUtil.applyOffsetToPose;
import static frc.robot.commands.AutoScoring.AutoScoreUtil.getAprilTagPose2d;
import static frc.robot.commands.AutoScoring.AutoScoreUtil.getPathFromWaypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants;
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
  private static final Timer apriltagTimer = new Timer();

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
    apriltagTimer.stop();
    apriltagTimer.reset();

  }

  public Command generateScoreNearestReefFace(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset) {
    return Commands.defer(() -> {
      Pose2d firstPoseDriveTo, secondPoseDriveTo;
      Pose2d closestAprilTagPose = closestAprilTag(m_DriveSubsystem.getPose());

      // straight behind the april tag by quite a bit
      firstPoseDriveTo = applyOffsetToPose(closestAprilTagPose, AutoScoreConstants.firstScoreLocationOffset);

      // left or right side of reef
      double sideOffset = (side.get() == Side.RIGHT) ? AutoScoreConstants.secondScoreLocationRightYOffset : AutoScoreConstants.secondScoreLocationLeftYOffset;
      secondPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(AutoScoreConstants.secondScoreLocationXOffset, sideOffset));

      // intake offset
      secondPoseDriveTo = applyOffsetToPose(secondPoseDriveTo, offset.get());

      System.out.println("align to nearest reef face, side is " + side.get() +
        "\napriltag pose is " + closestAprilTagPose.getX() + " " + closestAprilTagPose.getY() + " " + closestAprilTagPose.getRotation() +
        "\nfirstpose is " + firstPoseDriveTo.getX() + " " + firstPoseDriveTo.getY() +
        "\nsecondpose is " + secondPoseDriveTo.getX() + " " + secondPoseDriveTo.getY());

      return
        new SequentialCommandGroup(
          m_DriveSubsystem.driveToPose(firstPoseDriveTo),
          new ParallelDeadlineGroup(
            m_DriveSubsystem.driveToPoseSlowMode(secondPoseDriveTo),
            new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setpoint).asProxy()));
    }, Set.of());
  }

  public Command generateScoreNearestReefFaceOther(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset) {
    return Commands.defer(() -> {
      Pose2d firstPoseDriveTo, secondPoseDriveTo;
      Pose2d closestAprilTagPose = closestAprilTag(m_DriveSubsystem.getPose());

      // straight behind the april tag by quite a bit
      firstPoseDriveTo = applyOffsetToPose(closestAprilTagPose, AutoScoreConstants.firstScoreLocationOffset);

      // left or right side of reef
      double sideOffset = (side.get() == Side.RIGHT) ? AutoScoreConstants.secondScoreLocationRightYOffset : AutoScoreConstants.secondScoreLocationLeftYOffset;
      secondPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(AutoScoreConstants.secondScoreLocationXOffset, sideOffset));

      // intake offset
      secondPoseDriveTo = applyOffsetToPose(secondPoseDriveTo, offset.get());

      return
        getPathFromWaypoint(firstPoseDriveTo, m_DriveSubsystem).andThen(
          new ParallelDeadlineGroup(
            getPathFromWaypoint(secondPoseDriveTo, m_DriveSubsystem),
            new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setpoint)
          ),
          new IntakeClosedLoop(m_IntakeSubsystem, 0.8, true)
        );

    }, Set.of());
  }

  public void scheduleScoreNearestReefFace(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset, Supplier<Pose2d> closestTag) {
    Pose2d firstPoseDriveTo, secondPoseDriveTo;
    Pose2d closestAprilTagPose = closestTag.get();

    // straight behind the april tag by quite a bit
    firstPoseDriveTo = applyOffsetToPose(closestAprilTagPose, AutoScoreConstants.firstScoreLocationOffset);

    // left or right side of reef
    double sideOffset = (side.get() == Side.RIGHT) ? AutoScoreConstants.secondScoreLocationRightYOffset : AutoScoreConstants.secondScoreLocationLeftYOffset;
    secondPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(AutoScoreConstants.secondScoreLocationXOffset, sideOffset));

    // intake offset
    secondPoseDriveTo = applyOffsetToPose(secondPoseDriveTo, offset.get());

    System.out.println("align to nearest reef face, side is " + side.get() +
      "\napriltag pose is " + closestAprilTagPose.getX() + " " + closestAprilTagPose.getY() + " " + closestAprilTagPose.getRotation() +
      "\nfirstpose is " + firstPoseDriveTo.getX() + " " + firstPoseDriveTo.getY() +
      "\nsecondpose is " + secondPoseDriveTo.getX() + " " + secondPoseDriveTo.getY());

    Command commandToRun =
      new SequentialCommandGroup(
        m_DriveSubsystem.driveToPose(firstPoseDriveTo),
        new ParallelDeadlineGroup(
          m_DriveSubsystem.driveToPoseSlowMode(secondPoseDriveTo),
          new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setpoint).asProxy()));
    
    commandToRun.schedule();
  }
  

  public Command scoreSpecificTag(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset, int tag) {
    Pose2d firstPoseDriveTo, secondPoseDriveTo;
    Pose2d closestAprilTagPose = getAprilTagPose2d(tag);

    // straight behind the april tag by quite a bit
    firstPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(1,0));

    // left or right side of reef
    double sideOffset = (side.get() == Side.RIGHT) ? -0.2 : 0.2;
    secondPoseDriveTo = applyOffsetToPose(closestAprilTagPose, new Translation2d(0.5, sideOffset));

    // intake offset
    secondPoseDriveTo = applyOffsetToPose(secondPoseDriveTo, offset.get());

    System.out.println("align to nearest reef face, side is " + side.get() +
      "\napriltag pose is " + closestAprilTagPose.getX() + " " + closestAprilTagPose.getY() + " " + closestAprilTagPose.getRotation() +
      "\nfirstpose is " + firstPoseDriveTo.getX() + " " + firstPoseDriveTo.getY() +
      "\nsecondpose is " + secondPoseDriveTo.getX() + " " + secondPoseDriveTo.getY());

    return
      new SequentialCommandGroup(
        m_DriveSubsystem.driveToPoseSlowMode(firstPoseDriveTo),
        new ParallelCommandGroup(
          m_DriveSubsystem.driveToPoseSlowMode(secondPoseDriveTo),
          new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setpoint).asProxy()));
  }

}



