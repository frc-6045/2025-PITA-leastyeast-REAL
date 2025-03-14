package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmOpenLoop;
import frc.robot.commands.ElevatorCommands.ElevatorOpenLoop;
import frc.robot.commands.IntakeCommands.IntakeConditional;
import frc.robot.commands.IntakeCommands.IntakeIntake;
import frc.robot.commands.IntakeCommands.IntakeOpenLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class Bindings {
    public static boolean shift = false;

    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_godController,
        SwerveSubsystem m_driveSubsystem,
        ArmSubsystem m_Arm, 
        ElevatorSubsystem m_Elev, 
        IntakeSubsystem m_Intake,
        ClimbSubsystem m_ClimbSubsystem,
        LedSubsystem m_LedSubsystem) {


        /* Operator Controller bindings */

        //arm

        //intake
        //m_operatorController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController));
        //m_operatorController.rightTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController));

        m_operatorController.rightBumper().onTrue(
            new ParallelCommandGroup(
                new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.BARGE).asProxy(),
                new IntakeConditional(m_Intake, () -> {return (m_Arm.getAbsoluteEncoderPosition()+PositionConstants.kSketchyOffset)%1<0.5;}, false)
                )
        );

        //setpoints
        m_operatorController.y().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.HOME));
        m_operatorController.a().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.INTAKE));

        m_operatorController.pov(90).onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L1));
        m_operatorController.pov(270).onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L2));
        m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L3));
        m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L4));
        
        m_operatorController.x().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.ALGAE_HIGH));
        m_operatorController.b().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.ALGAE_LOW));

        //elev
        m_operatorController.pov(0).whileTrue(new ElevatorOpenLoop(m_Elev, true));
        m_operatorController.pov(180).whileTrue(new ElevatorOpenLoop(m_Elev, false));

        //misc
        m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {shift=true; System.out.println("SHIFT"); SmartDashboard.putBoolean("shift", shift);}));
        m_operatorController.leftBumper().onFalse(new InstantCommand(() -> {shift=false; System.out.println("NOT SHIFT"); SmartDashboard.putBoolean("shift", shift);}));
        // m_operatorController.leftBumper().whileTrue(
        //     new IntakeConditional(
        //         m_Intake, () -> {return (m_Arm.getAbsoluteEncoderPosition()+PositionConstants.kSketchyOffset)%1<0.5;}, 
        //         true
        //     )
        // );
        //m_operatorController.rightBumper().onTrue(new StopPIDArmAndElevator(m_Arm, m_Elevator)); // stop PID arm and elevator
        //m_operatorController.rightBumper().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.BARGE));

        /* Driver Controller non-drive bindings */

        m_driverController.a().whileTrue(new InstantCommand(() -> { m_LedSubsystem.setColor(23, 252, 3);}));

        m_driverController.leftTrigger(.15).whileTrue(new IntakeIntake(m_Intake, m_driverController, () -> {return IntakeSubsystem.coralDetected();}));
        m_driverController.rightTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_driverController));

        m_driverController.rightBumper().whileTrue(new ArmOpenLoop(m_Arm, true));
        m_driverController.leftBumper().whileTrue(new ArmOpenLoop(m_Arm, false));

        m_driverController.pov(0).whileTrue(new ClimbCommand(m_ClimbSubsystem, true));
        m_driverController.pov(180).whileTrue(new ClimbCommand(m_ClimbSubsystem, false));


        /* God Controller non-drive bindings */

        m_godController.leftTrigger().whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController));
        m_godController.rightTrigger().whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController));

        m_godController.rightBumper().whileTrue(new ArmOpenLoop(m_Arm, true));
        m_godController.leftBumper().whileTrue(new ArmOpenLoop(m_Arm, false));

        m_godController.b().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_Arm.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_Elev.getRelativeEncoderPosition());}));
        m_godController.x().onTrue(new StopPIDArmAndElevator(m_Arm, m_Elev));
        
        m_godController.y().onTrue(new PIDArmAndElevator(m_Arm, PositionConstants.kHomeArmPosition, m_Elev, PositionConstants.kHomeElevatorPosition));
        m_godController.a().onTrue(new PIDArmAndElevator(m_Arm, PositionConstants.kHumanArmPosition, m_Elev, PositionConstants.kHumanElevatorPosition));
        m_godController.pov(270).onTrue(new PIDArmAndElevator(m_Arm, PositionConstants.kL3ArmPosition, m_Elev, PositionConstants.kL3ElevatorPosition));
        m_godController.pov(90).onTrue(new PIDArmAndElevator(m_Arm, PositionConstants.kL4ArmPosition, m_Elev, PositionConstants.kL4ElevatorPosition));

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

    public static boolean isShift() {
        return shift;
    }

    public static void configureDrivetrain(SwerveSubsystem m_DriveSubsystem, CommandXboxController m_driverController) {
            /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(()->{return -m_driverController.getRightX();})
                                                            .deadband(ControllerConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  //SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  //                                                                                           m_driverController::getRightY)
  //                                                         .headingWhile(true);

  /**
   * Clones the angular velocity input stream and converts it to a robotRelative input stream.
   */
  //SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
  //                                                           .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(ControllerConstants.DEADBAND)
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
        //m_driverController.x().onTrue(Commands.runOnce(m_DriveSubsystem::addFakeVisionReading));
        //m_driverController.b().whileTrue(
        //    m_DriveSubsystem.driveToPose(
        //        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
        //                        );
        //m_driverController.start().whileTrue(Commands.none());
        //m_driverController.back().whileTrue(Commands.none());
        //m_driverController.leftBumper().whileTrue(Commands.runOnce(m_DriveSubsystem::lock, m_DriveSubsystem).repeatedly());
        //m_driverController.rightBumper().onTrue(Commands.none());
        }
  }

}
