// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// subsystems
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmCommands.HoldArm;
import frc.robot.commands.ElevatorCommands.HoldElevator;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

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
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
  private Autos m_Autos;
  
  public final SwerveSubsystem m_DriveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  // define controllers
  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  private final CommandXboxController m_godController =
      new CommandXboxController(ControllerConstants.kGodControllerPort);
  private final XboxController m_autScoringTestController =
      new XboxController(3);   

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Autos = new Autos(m_DriveSubsystem, m_IntakeSubsystem, m_ElevatorSubsystem, m_ArmSubsystem);
    Bindings.InitBindings(m_operatorController, m_driverController, m_godController, m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, m_ClimbSubsystem, m_LedSubsystem);
    Bindings.configureDrivetrain(m_DriveSubsystem, m_driverController);
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
    m_ElevatorSubsystem.setDefaultCommand(new HoldElevator(m_ElevatorSubsystem));
    DriverStation.silenceJoystickConnectionWarning(true);



  // new Trigger(() -> m_autScoringTestController.getLeftStickButton() && m_autScoringTestController.getAButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L1, Side.LEFT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getLeftStickButton() && m_autScoringTestController.getBButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L2, Side.LEFT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getLeftStickButton() && m_autScoringTestController.getXButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L3, Side.LEFT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getLeftStickButton() && m_autScoringTestController.getYButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L4, Side.LEFT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getRightStickButton() && m_autScoringTestController.getAButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L1, Side.RIGHT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getRightStickButton() && m_autScoringTestController.getBButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L2, Side.RIGHT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getRightStickButton() && m_autScoringTestController.getXButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L3, Side.RIGHT, new Translation2d())
  //   );
  //   new Trigger(() -> m_autScoringTestController.getRightStickButton() && m_autScoringTestController.getYButton()).onTrue(
  //     new AutoScoreNearestReefFace(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem, Setpoints.L4, Side.RIGHT, new Translation2d())
  //   );
  }
  
  
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }


  // public Command ConstructAutoScoreCommand(PositionConstants.Setpoints setPoint, AutoScoreConstants.Side side, Translation2d offset) {
  //     //calculate first pose to quickly travel too
  //     //note that the offset is currently not utilized but can be done by using the shift robot relative command
  //     Translation2d shiftBackward = new Translation2d(-1, 0);
  //     Pose2d nearestPole = m_DriveSubsystem.getNearestPole(side);
  //     Pose2d initalPoseToPlanTo = m_DriveSubsystem.shiftPoseRobotRelative(nearestPole, shiftBackward);
  //     System.out.println("Initial Target Pose: " + initalPoseToPlanTo.getX() + ", " + initalPoseToPlanTo.getY()+ ", " + nearestPole.getRotation().getRadians());
  //     Command planToPathFast = m_DriveSubsystem.driveToPose(initalPoseToPlanTo);
      
  //     Command MoveArmAndElevator;
  //     if(setPoint.equals(PositionConstants.Setpoints.L1)) {
  //       MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L1);
  //     } else if(setPoint.equals(PositionConstants.Setpoints.L2)) {
  //       MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L2);
  //     } else if(setPoint.equals(PositionConstants.Setpoints.L3)) {
  //       MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L3);
  //     } else if(setPoint.equals(PositionConstants.Setpoints.L4)) {
  //       MoveArmAndElevator = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.L4);
  //     } else {
  //       return new PrintCommand("you messed up the construct auto command");
  //     }
  //     Command planToPathSlow = m_DriveSubsystem.driveToPoseSlowMode(nearestPole);
  //     Command CoralSpit = new ParallelDeadlineGroup(new WaitCommand(0.5), new IntakeOpenLoop(m_IntakeSubsystem, m_operatorController));
  //     // Command toHome = new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, Setpoints.HOME);
  //     System.out.println("Initial Target Pose: " + nearestPole.getX() + ", " + nearestPole.getY() + ", " + nearestPole.getRotation().getRadians());
  //     return planToPathFast.andThen(new ParallelDeadlineGroup(planToPathSlow, MoveArmAndElevator)).andThen(CoralSpit);
  // }
}