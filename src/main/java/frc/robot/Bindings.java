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
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbWristOpenLoop;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmOpenLoop;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.AutoScoring.AlignToReefTagRelative;
import frc.robot.commands.AutoScoring.AutoScoreCommands;
import frc.robot.commands.AutoScoring.TeleopScoreNearestReefFace;
import frc.robot.commands.ElevatorCommands.ElevatorOpenLoop;
import frc.robot.commands.ElevatorCommands.PIDElevatorCommand;
import frc.robot.commands.IntakeCommands.IntakeConditional;
import frc.robot.commands.IntakeCommands.IntakeIntake;
import frc.robot.commands.IntakeCommands.IntakeOpenLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class Bindings {
    public static boolean operatorShift = false;
    public static boolean ledstate = false;

    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_testController,        SwerveSubsystem m_driveSubsystem,
        ArmSubsystem m_Arm, 
        ElevatorSubsystem m_Elev, 
        IntakeSubsystem m_Intake,
        ClimbSubsystem m_ClimbSubsystem,
        LedSubsystem m_LedSubsystem,
        ClimbWristSubsystem m_Wrist) {

        AutoScoreCommands m_AutoScoreCommands = 
            new AutoScoreCommands(m_driveSubsystem, m_Arm, m_Elev, m_Intake);

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

        m_operatorController.rightBumper().onTrue(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    Commands.none().until(() -> m_Elev.getRelativeEncoderPosition()<-50),
                    new PIDArmCommand(m_Arm, PositionConstants.kBargeArm).asProxy()),
                new PIDElevatorCommand(m_Elev, PositionConstants.kBargeElev).asProxy(),
                new IntakeConditional(m_Intake, () -> {return m_Arm.getSketchyOffsettedPosition()<0.54
                    ;}, true, 0.9)
            ));

        // Setpoints
        m_operatorController.y().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.HOME));
        m_operatorController.a().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.INTAKE));

        m_operatorController.pov(90).onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L1));
        m_operatorController.pov(270).onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L2));
        m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L3));
        m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.L4));
        
    
        m_operatorController.x().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.ALGAE_HIGH));
        m_operatorController.b().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.ALGAE_LOW));

        // Elevator
        m_operatorController.pov(0).whileTrue(new ElevatorOpenLoop(m_Elev, true));
        m_operatorController.pov(180).whileTrue(new ElevatorOpenLoop(m_Elev, false));

        // Shift (elevator bottom switch override)
        m_operatorController.leftBumper().onTrue(new InstantCommand(()->{if (m_Elev.getBottomLimitSwitchState()) m_Elev.zeroEncoder();}));

        //m_operatorController.start().whileTrue(new ClimbCommand(m_ClimbSubsystem, false));


        /* Driver Controller bindings */

        m_driverController.rightTrigger(.15).whileTrue(new IntakeIntake(m_Intake, m_driverController, () -> {return m_Intake.coralDetected();}, MotorConstants.kIntakeMotorSpeed));
        m_driverController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_driverController, MotorConstants.kIntakeMotorSpeed));

        m_driverController.rightBumper().whileTrue(new ArmOpenLoop(m_Arm, true));
        m_driverController.leftBumper().whileTrue(new ArmOpenLoop(m_Arm, false));

        // m_driverController.pov(0).whileTrue(new ClimbCommand(m_ClimbSubsystem, true));
        // m_driverController.pov(180).whileTrue(new ClimbCommand(m_ClimbSubsystem, false));

        // m_driverController.pov(90).whileTrue(new ClimbWristOpenLoop(m_Wrist, MotorConstants.kWristMotorSpeed));
        // m_driverController.pov(270).whileTrue(new ClimbWristOpenLoop(m_Wrist, -MotorConstants.kWristMotorSpeed));
       // m_driverController.a().onTrue(new ClimbClosedLoop(m_ClimbSubsystem, -23452,5));

        m_driverController.start().onTrue(Commands.runOnce(() -> m_driveSubsystem.zeroGyroWithAlliance()).alongWith(new PrintCommand("resest heading")));
        
        m_driverController.x().whileTrue(new AlignToReefTagRelative(Side.LEFT, m_driveSubsystem, m_driverController, () -> {return m_Intake.getAlignOffsetLimelightTX();}));
		m_driverController.b().whileTrue(new AlignToReefTagRelative(Side.RIGHT, m_driveSubsystem, m_driverController, () -> {return m_Intake.getAlignOffsetLimelightTX();}));
        
        m_driverController.y().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.LOLLIPOP));
        
        
        m_driverController.a().onTrue(new ConditionalCommand(Commands.runOnce(()->{m_LedSubsystem.set(-0.49);}), Commands.runOnce(()->{m_LedSubsystem.set(-0.7);}), ()->{return ledstate;}).alongWith(new InstantCommand(()->ledstate = !ledstate)));

        /* Test Controller bindings */

        m_testController.a().onTrue(m_driveSubsystem.driveToFirstAutoScorePose(Side.LEFT));
        m_testController.y().onTrue(new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.HOME));
        m_testController.b().onTrue(
            m_AutoScoreCommands.scoreNearestReefFaceOther(
                Setpoints.L3,
                ()->{return Side.LEFT;},
                ()->{return m_Intake.getAlignOffset();}
            )
        );
        m_testController.x().onTrue(
            new TeleopScoreNearestReefFace(
                m_driveSubsystem, m_Arm, m_Elev, m_Intake,
                Setpoints.L3,
                ()->{return Side.LEFT;},
                ()->{return m_Intake.getAlignOffset();}
            )
        );
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
