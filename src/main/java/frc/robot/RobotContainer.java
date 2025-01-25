// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;

// subsystems
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.ArmCommands.StopPIDArmCommand;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//Quinn


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public static int BumperPressed = 0;
  private static boolean bIntakeToggle = true;

  // define controllers
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // operator triggers control coral intake
    m_operatorController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));
    m_operatorController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));

    // arm
    //m_driverController.rightTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, true, m_driverController));
    //m_driverController.leftTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, false, m_driverController));
    
   //m_operatorController.b().onTrue(new InstantCommand(() -> {System.out.println(m_ArmSubsystem.getAbsoluteEncoderPosition());}));
    m_operatorController.a().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm

    // setpoints (y: home, b: human)2   
    //m_operatorController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
    //m_operatorController.b().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
//Quinn's Crap

    m_operatorController.pov(90).whileTrue(new ArmCommand(m_ArmSubsystem, true, m_operatorController));
    m_operatorController.pov(270).whileTrue(new ArmCommand(m_ArmSubsystem, false, m_operatorController));

    
    // d pad controls elevator
    m_operatorController.pov(0).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, true));
    m_operatorController.pov(180).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, false));
      
    // paddles will have setpoints 1-8

  }
  // /** 
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //  // return Autos.exampleAuto(m_sparkFlexTesterSubsystem);
  // }
}
