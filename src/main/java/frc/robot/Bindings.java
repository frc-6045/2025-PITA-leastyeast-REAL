package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.IntakeCommands.IntakeConditional;
import frc.robot.commands.IntakeCommands.IntakeIntake;
import frc.robot.commands.IntakeCommands.IntakeOpenLoop;
import frc.robot.commands.IntakeCommands.IntakeVelocityFromDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class Bindings {
    public static boolean operatorShift = false;
    public static boolean ledstate = false;

    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_testController,        SwerveSubsystem m_driveSubsystem,
        IntakeSubsystem m_Intake) {

        
        /* Operator Controller bindings */

        // Intake
        // m_operatorController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController, MotorConstants.kOperatorIntakeMotorSpeed));
        // m_operatorController.rightTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController, MotorConstants.kOperatorIntakeMotorSpeed));
        m_operatorController.rightTrigger(.15).whileTrue(new IntakeIntake(m_Intake, m_operatorController, () -> {return m_Intake.coralDetected();}, MotorConstants.kIntakeMotorSpeed));
        m_operatorController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController, MotorConstants.kIntakeMotorSpeed));


        // Barge Toss
        // m_operatorController.rightBumper().onTrue(
        //     new SequentialCommandGroup(
        //         new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.ALGAE_HIGH).asProxy(),
        //         new ParallelCommandGroup(
        //             new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.BARGE).asProxy(),
        //             new IntakeConditional(m_Intake, () -> {return m_Arm.getSketchyOffsettedPosition()<0.6;}, true, 0.9)
        //         )
        //     )
        // );

     
        

   


        /* Driver Controller bindings */

        m_driverController.rightTrigger(.15).whileTrue(new IntakeIntake(m_Intake, m_driverController, () -> {return m_Intake.coralDetected();}, MotorConstants.kIntakeMotorSpeed));
        m_driverController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_driverController, MotorConstants.kIntakeMotorSpeed));


        // m_driverController.pov(0).whileTrue(new ClimbCommand(m_ClimbSubsystem, true));
        // m_driverController.pov(180).whileTrue(new ClimbCommand(m_ClimbSubsystem, false));

        // m_driverController.pov(90).whileTrue(new ClimbWristOpenLoop(m_Wrist, MotorConstants.kWristMotorSpeed));
        // m_driverController.pov(270).whileTrue(new ClimbWristOpenLoop(m_Wrist, -MotorConstants.kWristMotorSpeed));
       // m_driverController.a().onTrue(new ClimbClosedLoop(m_ClimbSubsystem, -23452,5));

        m_driverController.start().onTrue(Commands.runOnce(() -> m_driveSubsystem.zeroGyroWithAlliance()).alongWith(new PrintCommand("resest heading")));

        // Test controller - intake velocity testing with dashboard slider
        m_driverController.a().whileTrue(new IntakeVelocityFromDashboard(m_Intake));

    }

    public static boolean getOperatorShiftPressed() {
        return operatorShift;
    }

    public static void configureDrivetrain(SwerveSubsystem m_DriveSubsystem, CommandXboxController m_driverController) {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                        () -> m_driverController.getLeftY() * -1,
                                                                        () -> m_driverController.getLeftX() * -1)
                                                                    .withControllerRotationAxis(()->{return -m_driverController.getRightX();})
                                                                    .deadband(ControllerConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

        Command driveFieldOrientedAnglularVelocity = m_DriveSubsystem.driveFieldOriented(driveAngularVelocity);
        m_DriveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }
}
