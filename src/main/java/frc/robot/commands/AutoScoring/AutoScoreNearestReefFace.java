// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants;
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
public class AutoScoreNearestReefFace extends InstantCommand {

  SwerveSubsystem m_DriveSubsystem;
  ArmSubsystem m_ArmSubsystem; 
  ElevatorSubsystem m_ElevatorSubsystem; 
  IntakeSubsystem m_IntakeSubsystem; 
  PositionConstants.Setpoints setPoint; 
  Supplier<Translation2d> offset;
  Supplier<Side> side;


  public AutoScoreNearestReefFace(SwerveSubsystem m_DriveSubsystem, 
  ArmSubsystem m_ArmSubsystem, ElevatorSubsystem m_ElevatorSubsystem, 
  IntakeSubsystem m_IntakeSubsystem, 
  PositionConstants.Setpoints setPoint, 
  Supplier<Side> side, 
  Supplier<Translation2d> offset) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_ElevatorSubsystem = m_ElevatorSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.setPoint = setPoint;
    this.side = side;
    this.offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Command commandToRun;

    Command MoveArmAndElevator;
    if(setPoint.equals(PositionConstants.Setpoints.L1)) {
      MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L1).asProxy();
    } else if(setPoint.equals(PositionConstants.Setpoints.L2)) {
      MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L2).asProxy();
    } else if(setPoint.equals(PositionConstants.Setpoints.L3)) {
      MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L3).asProxy();
    } else if(setPoint.equals(PositionConstants.Setpoints.L4)) {
      MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L4).asProxy();
    } else {
      MoveArmAndElevator = new PrintCommand("your auto score command is broken cause this is an invalid setpoint");
    }
    commandToRun = m_DriveSubsystem.driveToFirstAutoScorePose(side.get()).andThen( 
    new ParallelDeadlineGroup(
      m_DriveSubsystem.driveToSecondAutoScorePose(side.get(), offset.get()), 
      MoveArmAndElevator),
    new IntakeClosedLoop(m_IntakeSubsystem, 0.8, true)
    );

    System.out.println("should be running");
    // commandToRun.addRequirements(m_ArmSubsystem, m_DriveSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
    commandToRun.schedule();
  }
}
