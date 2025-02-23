// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;

// subsystems
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.ArmCommands.HoldArm;
import frc.robot.commands.ElevatorCommands.HoldElevator;
import frc.robot.commands.IntakeCommands.IntakeAuto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private Autos m_Autos;
  
  public final SwerveSubsystem m_DriveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  // define controllers
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_godController =
      new CommandXboxController(OperatorConstants.kGodControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("coralSpit", new IntakeAuto(m_IntakeSubsystem, 1, false));
    NamedCommands.registerCommand("coralIntake", new IntakeAuto(m_IntakeSubsystem, 1, true));
    NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
    NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
    NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
    NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
    NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
    NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
    m_Autos = new Autos(m_DriveSubsystem, m_IntakeSubsystem, m_ElevatorSubsystem, m_ArmSubsystem);
    Bindings.InitBindings(m_operatorController, m_driverController, m_godController, m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
    Bindings.configureDrivetrain(m_DriveSubsystem, m_driverController);
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
    m_ElevatorSubsystem.setDefaultCommand(new HoldElevator(m_ElevatorSubsystem));
    DriverStation.silenceJoystickConnectionWarning(true);
    
    
  }
  
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }
}
