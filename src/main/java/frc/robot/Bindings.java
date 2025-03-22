package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmOpenLoop;
import frc.robot.commands.AutoScoring.AlignToReefTagRelative;
import frc.robot.commands.AutoScoring.AutoScoreCommands;
import frc.robot.commands.AutoScoring.TeleopScoreNearestReefFace;
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
    public static boolean operatorShift = false;
    public static Side reefScoreLeftOrRight = null;

    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_testController,
        SwerveSubsystem m_driveSubsystem,
        ArmSubsystem m_Arm, 
        ElevatorSubsystem m_Elev, 
        IntakeSubsystem m_Intake,
        ClimbSubsystem m_ClimbSubsystem,
        LedSubsystem m_LedSubsystem) {

        AutoScoreCommands m_AutoScoreCommands = 
            new AutoScoreCommands(m_driveSubsystem, m_Arm, m_Elev, m_Intake);


        /* Operator Controller bindings */

        //intake
        m_operatorController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController, MotorConstants.kOperatorIntakeMotorSpeed));
        m_operatorController.rightTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_operatorController, MotorConstants.kOperatorIntakeMotorSpeed));

        // barge toss
        m_operatorController.rightBumper().onTrue(
            new ParallelCommandGroup(
                new PIDArmAndElevator(m_Arm, m_Elev, Setpoints.BARGE).asProxy(),
                new IntakeConditional(m_Intake, () -> {return m_Arm.getSketchyOffsettedPosition()<0.5;}, false, 2)
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

        //shift (elevator bottom switch override)
        //m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {operatorShift=true; System.out.println("SHIFT"); SmartDashboard.putBoolean("shift", operatorShift);}));
        //m_operatorController.leftBumper().onFalse(new InstantCommand(() -> {operatorShift=false; System.out.println("NOT SHIFT"); SmartDashboard.putBoolean("shift", operatorShift);}));
        m_operatorController.leftBumper().onTrue(new InstantCommand(()->{if (m_Elev.getBottomLimitSwitchState()) m_Elev.zeroEncoder();}));


        /* Driver Controller bindings */

        //m_driverController.y().whileTrue(m_LedSubsystem.runPattern(LEDPattern.solid(Color.kRed)));
        m_driverController.a().onTrue((Commands.runOnce(m_driveSubsystem::zeroGyro)));

        m_driverController.rightTrigger(.15).whileTrue(new IntakeIntake(m_Intake, m_driverController, () -> {return m_Intake.coralDetected();}, MotorConstants.kIntakeMotorSpeed));
        m_driverController.leftTrigger(.15).whileTrue(new IntakeOpenLoop(m_Intake, m_driverController, MotorConstants.kIntakeMotorSpeed));

        m_driverController.rightBumper().whileTrue(new ArmOpenLoop(m_Arm, true));
        m_driverController.leftBumper().whileTrue(new ArmOpenLoop(m_Arm, false));

        m_driverController.pov(0).whileTrue(new ClimbCommand(m_ClimbSubsystem, true));
        m_driverController.pov(180).whileTrue(new ClimbCommand(m_ClimbSubsystem, false));

        m_driverController.start().onTrue(Commands.runOnce(() -> m_driveSubsystem.zeroGyroWithAlliance()).alongWith(new PrintCommand("resest heading")));

        //reef score left or right
        m_driverController.leftStick().onTrue(new InstantCommand(()->{reefScoreLeftOrRight = Side.LEFT;}));
        m_driverController.leftStick().onFalse(new InstantCommand(()->{reefScoreLeftOrRight = null;}));
        m_driverController.rightStick().onTrue(new InstantCommand(()->{reefScoreLeftOrRight = Side.RIGHT;}));
        m_driverController.rightStick().onFalse(new InstantCommand(()->{reefScoreLeftOrRight = null;}));
        
        m_driverController.x().whileTrue(new AlignToReefTagRelative(Side.LEFT, m_driveSubsystem, m_driverController, () -> {return m_Intake.getAlignOffsetLimelightTX();}));
		m_driverController.b().whileTrue(new AlignToReefTagRelative(Side.RIGHT, m_driveSubsystem, m_driverController, () -> {return m_Intake.getAlignOffsetLimelightTX();}));

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

    public static Side getReefScoreLeftOrRight() {
        return reefScoreLeftOrRight;
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
