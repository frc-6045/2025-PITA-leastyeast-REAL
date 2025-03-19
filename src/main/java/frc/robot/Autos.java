package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.AutoScoring.AutoScoreCommands;
import frc.robot.commands.AutoScoring.TeleopScoreNearestReefFace;
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

        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("coralSpinNormal", new IntakeClosedLoop(intake, 1, false)); //score for L3, L4, algae out
        NamedCommands.registerCommand("coralSpinOther", new IntakeClosedLoop(intake, 0.5, true)); //score for L2, L1
        NamedCommands.registerCommand("algaeIn", new IntakeClosedLoop(intake, 2, true)); //algae in
        NamedCommands.registerCommand("coralIntake", new IntakeIntakeClosedLoop(intake, ()->{return intake.coralDetected();}, MotorConstants.kIntakeMotorSpeed));
        NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(arm, elev, Setpoints.L1).asProxy());
        NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(arm, elev, Setpoints.L2).asProxy());
        NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(arm, elev, Setpoints.L3).asProxy());
        NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(arm, elev, Setpoints.L4).asProxy());
        NamedCommands.registerCommand("algaeHigh", new PIDArmAndElevator(arm, elev, Setpoints.ALGAE_HIGH).asProxy());
        NamedCommands.registerCommand("algaeLow", new PIDArmAndElevator(arm, elev, Setpoints.ALGAE_LOW).asProxy());
        NamedCommands.registerCommand("barge", new ParallelCommandGroup(
                new PIDArmAndElevator(arm, elev, Setpoints.BARGE).asProxy(),
                new IntakeConditional(intake, () -> {return arm.getSketchyOffsettedPosition()<0.5;}, false)
                ));
        NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(arm, elev, Setpoints.INTAKE).asProxy());
        NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(arm, elev, Setpoints.HOME).asProxy());
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
        NamedCommands.registerCommand("alignPrint", Commands.print("The robot is aligning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
        // NamedCommands.registerCommand("scoreCoralL1",
        //     new SequentialCommandGroup(
        //         new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition).asProxy(),
        //         new AlignToReefTagRelative(m_SwerveSubsystem),
        //         new IntakeClosedLoop(m_IntakeSubsystem, 1, false)
        //     )
        // );


        // Autos //

        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("Do Nothing", new InstantCommand(() -> {System.out.println("hi");}));
        autoChooser.addOption("test", AutoBuilder.buildAuto("New New Auto"));
        autoChooser.addOption("3PieceIKJPolesLimelight", AutoBuilder.buildAuto("3PieceIKJPolesLimelight"));
        autoChooser.addOption("(Left)Coral2AlgaeHGHKJPoles", AutoBuilder.buildAuto("Coral2AlgaeHGHKJPoles"));
        autoChooser.addOption("do not run-(Right)Coral2AlgaeHGHEFPoles", AutoBuilder.buildAuto("Coral2AlgaeHGHEFPoles"));
        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
