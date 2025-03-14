// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  SwerveSubsystem m_DriveSubsystem;
  ArmSubsystem m_ArmSubsystem; 
  ElevatorSubsystem m_ElevatorSubsystem; 
  IntakeSubsystem m_IntakeSubsystem; 


  public AutoScoreCommands(
      SwerveSubsystem m_DriveSubsystem, 
      ArmSubsystem m_ArmSubsystem, ElevatorSubsystem m_ElevatorSubsystem, 
      IntakeSubsystem m_IntakeSubsystem) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_ElevatorSubsystem = m_ElevatorSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
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

  public Command scoreSpecificReefPole(Setpoints setPoint, Pose2d pole, Supplier<Translation2d> offset) {
    return m_DriveSubsystem.driveToFirstAutoScorePose(pole).andThen( 
      new ParallelDeadlineGroup(
        m_DriveSubsystem.driveToSecondAutoScorePose(pole, offset.get()), 
        new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, setPoint)),
      new IntakeClosedLoop(m_IntakeSubsystem, 0.8, true)
      );
    //return Commands.print("hi");
  }
}
