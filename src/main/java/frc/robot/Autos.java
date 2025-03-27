package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.AutoScoring.AutoScoreCommands;
import frc.robot.commands.ElevatorCommands.PIDElevatorCommand;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
import frc.robot.commands.IntakeCommands.IntakeConditional;
import frc.robot.commands.IntakeCommands.IntakeIntakeClosedLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final IntakeSubsystem m_Intake;
    private final ElevatorSubsystem m_Elevator;
    private final ArmSubsystem m_Arm;
    private SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_SwerveSubsystem = drive;
        m_Intake = intake;
        m_Elevator = elev;
        m_Arm = arm;
        AutoScoreCommands m_AutoScoreCommands = 
            new AutoScoreCommands(m_SwerveSubsystem, m_Arm, m_Elevator, m_Intake);

        // Named Commands //
        NamedCommands.registerCommand("coralSpinNormal", new InstantCommand(() -> {})); //score for L3, L4, algae out
        NamedCommands.registerCommand("coralSpinOther", new InstantCommand(() -> {})); //score for L2, L1
        NamedCommands.registerCommand("algaeInOne", new InstantCommand(() -> {})); //algae in
        NamedCommands.registerCommand("algaeInTwo", new InstantCommand(() -> {})); //algae in
        NamedCommands.registerCommand("coralIntake", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("coralL1", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("coralL2", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("coralL3", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("coralL4", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("algaeHigh", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("algaeLow", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("barge", new InstantCommand(() -> {}));
        // NamedCommands.registerCommand("bargeNew",
        //     new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //             Commands.none().until(() -> elev.getRelativeEncoderPosition()<-50),
        //             new PIDArmCommand(arm, PositionConstants.kBargeArm).asProxy()),
        //         new PIDElevatorCommand(elev, PositionConstants.kBargeElev),
        //         new IntakeConditional(intake, () -> {return arm.getSketchyOffsettedPosition()<0.6;}, true, 0.9)
        //     ));
        NamedCommands.registerCommand("coralIntakeSetpoint", new InstantCommand(() -> {}));
        NamedCommands.registerCommand("homePosition", new InstantCommand(() -> {}));
        // NamedCommands.registerCommand("alignToReefScoreL3Left", 
        //     m_AutoScoreCommands.scoreNearestReefFace(
        //         Setpoints.L3,
        //         ()->{return Side.LEFT;},
        //         ()->{return intake.getAlignOffset();}
        //     )
        // );
        // NamedCommands.registerCommand("alignToReefScoreL3Right", 
        //     m_AutoScoreCommands.scoreNearestReefFace(
        //         Setpoints.L3,
        //         ()->{return Side.RIGHT;},
        //         ()->{return intake.getAlignOffset();}
        //     )
        // );
        // NamedCommands.registerCommand("alignToReefScoreL4Left", 
        //     m_AutoScoreCommands.scoreNearestReefFace(
        //         Setpoints.L4,
        //         ()->{return Side.LEFT;},
        //         ()->{return intake.getAlignOffset();}
        //     )
        // );
        // NamedCommands.registerCommand("alignToReefScoreL4Right", 
        //     m_AutoScoreCommands.scoreNearestReefFace(
        //         Setpoints.L4,
        //         ()->{return Side.RIGHT;},
        //         ()->{return intake.getAlignOffset();}
        //     )
        // );

        // Autos //
        autoChooser = new SendableChooser<Command>();
        // autoChooser.addOption("test", AutoBuilder.buildAuto("New New Auto"));
        // autoChooser.addOption("3PieceIKJPolesLimelight", AutoBuilder.buildAuto("3PieceIKJPolesLimelight"));
        autoChooser.addOption("coral l4 then 2 algae", AutoBuilder.buildAuto("Coral2AlgaeHGHIJPoles"));
        autoChooser.addOption("drive forward", AutoBuilder.buildAuto("driveForward"));
        autoChooser.addOption("test", AutoBuilder.buildAuto("testingpath"));
        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
