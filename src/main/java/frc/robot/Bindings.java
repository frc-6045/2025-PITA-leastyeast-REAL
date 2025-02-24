package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmOpenLoop;
import frc.robot.commands.ElevatorCommands.ElevatorOpenLoop;
import frc.robot.commands.IntakeCommands.IntakeOpenLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class Bindings {
    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_godController,
        SwerveSubsystem m_driveSubsystem,
        ArmSubsystem m_ArmSubsystem, 
        ElevatorSubsystem m_ElevatorSubsystem, 
        IntakeSubsystem m_IntakeSubsystem) {


        /* Operator Controller bindings */

        //intake
        m_operatorController.leftTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_operatorController));
        m_operatorController.rightTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_operatorController));
        
        //misc
        m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_ArmSubsystem.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_ElevatorSubsystem.getRelativeEncoderPosition());}));
        m_operatorController.x().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm and elevator
        
        //setpoints
        m_operatorController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
        m_operatorController.a().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));

        m_operatorController.b().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
        m_operatorController.pov(270).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
        m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
        
        //elev
        m_operatorController.pov(0).whileTrue(new ElevatorOpenLoop(m_ElevatorSubsystem, true));
        m_operatorController.pov(180).whileTrue(new ElevatorOpenLoop(m_ElevatorSubsystem, false));


        /* Driver Controller arm and intake */

        m_driverController.leftTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_driverController));
        m_driverController.rightTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_driverController));

        m_driverController.rightBumper().whileTrue(new ArmOpenLoop(m_ArmSubsystem, true));
        m_driverController.leftBumper().whileTrue(new ArmOpenLoop(m_ArmSubsystem, false));


        /* Controller with both driver and operator functions */

        m_godController.leftTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_operatorController));
        m_godController.rightTrigger().whileTrue(new IntakeOpenLoop(m_IntakeSubsystem, m_operatorController));

        m_godController.rightBumper().whileTrue(new ArmOpenLoop(m_ArmSubsystem, true));
        m_godController.leftBumper().whileTrue(new ArmOpenLoop(m_ArmSubsystem, false));

        m_godController.b().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_ArmSubsystem.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_ElevatorSubsystem.getRelativeEncoderPosition());}));
        m_godController.x().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm and elevator
        
        m_godController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
        m_godController.a().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
        m_godController.pov(270).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        m_godController.pov(90).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));

        // m_DriveSubsystem.setDefaultCommand(
        // new RunCommand(
        //     () -> m_DriveSubsystem.drive( 
        //         MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.30)+MathUtil.applyDeadband(-m_godController.getLeftY(), 0.30), 
        //         MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.30)+MathUtil.applyDeadband(-m_godController.getLeftX(), 0.30),
        //         MathUtil.applyDeadband(-m_driverController.getRightX(), 0.30)+MathUtil.applyDeadband(-m_godController.getRightX(), 0.30),
        //         true),
        //     m_DriveSubsystem)
        // );
    }

    public static void configureDrivetrain(SwerveSubsystem m_DriveSubsystem, CommandXboxController m_driverController) {
            /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(()->{return -m_driverController.getRightX();})
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  //SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  //                                                                                           m_driverController::getRightY)
  //                                                         .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  //SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
  //                                                           .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *2))
                                                                               .headingWhile(true);
        //Command driveFieldOrientedDirectAngle      = m_DriveSubsystem.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = m_DriveSubsystem.driveFieldOriented(driveAngularVelocity);
        //Command driveRobotOrientedAngularVelocity  = m_DriveSubsystem.driveFieldOriented(driveRobotOriented);
        //Command driveSetpointGen = m_DriveSubsystem.driveWithSetpointGeneratorFieldRelative(
        //    driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard      = m_DriveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
        //Command driveFieldOrientedAnglularVelocityKeyboard = m_DriveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
        //Command driveSetpointGenKeyboard = m_DriveSubsystem.driveWithSetpointGeneratorFieldRelative(
        //    driveDirectAngleKeyboard);

        if (RobotBase.isSimulation())
        {
        m_DriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else
        {
        m_DriveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (Robot.isSimulation())
        {
        m_driverController.start().onTrue(Commands.runOnce(() -> m_DriveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        m_driverController.button(1).whileTrue(m_DriveSubsystem.sysIdDriveMotorCommand());

        }
        if (DriverStation.isTest())
        {
        m_DriveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

        m_driverController.x().whileTrue(Commands.runOnce(m_DriveSubsystem::lock, m_DriveSubsystem).repeatedly());
        m_driverController.y().whileTrue(m_DriveSubsystem.driveToDistanceCommand(1.0, 0.2));
        m_driverController.start().onTrue((Commands.runOnce(m_DriveSubsystem::zeroGyro)));
        m_driverController.back().whileTrue(m_DriveSubsystem.centerModulesCommand());
        m_driverController.leftBumper().onTrue(Commands.none());
        m_driverController.rightBumper().onTrue(Commands.none());
        } else
        {
        m_driverController.a().onTrue((Commands.runOnce(m_DriveSubsystem::zeroGyro)));
        m_driverController.x().onTrue(Commands.runOnce(m_DriveSubsystem::addFakeVisionReading));
        m_driverController.b().whileTrue(
            m_DriveSubsystem.driveToPose(
                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                );
        m_driverController.start().whileTrue(Commands.none());
        m_driverController.back().whileTrue(Commands.none());
        m_driverController.leftBumper().whileTrue(Commands.runOnce(m_DriveSubsystem::lock, m_DriveSubsystem).repeatedly());
        m_driverController.rightBumper().onTrue(Commands.none());
        }
  }

}
