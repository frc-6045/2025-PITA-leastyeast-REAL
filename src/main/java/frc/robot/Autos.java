package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
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

        // Named Commands
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("coralSpit", new IntakeClosedLoop(m_Intake, 1, false));
        NamedCommands.registerCommand("coralIntake", new IntakeClosedLoop(m_Intake, 0.5, true));
        NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.L1).asProxy());
        NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.L2).asProxy());
        NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.L3).asProxy());
        NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.L4).asProxy());
        NamedCommands.registerCommand("algaeHigh", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.ALGAE_HIGH));
        NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.INTAKE).asProxy());
        NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(m_Arm, m_Elevator, Setpoints.HOME).asProxy());
        NamedCommands.registerCommand("alignToReefLeft", m_SwerveSubsystem.driveToPose(new Pose2d(6.761, 3.779, Rotation2d.fromDegrees(0))).andThen(AutoBuilder.buildAuto("HPSToGPole2Auto")));
        NamedCommands.registerCommand("alignToReefRight", m_SwerveSubsystem.driveToPose(new Pose2d(6.761, 3.779, Rotation2d.fromDegrees(0))).andThen(AutoBuilder.buildAuto("HPSToGPole2Auto")));
        NamedCommands.registerCommand("alignPrint", Commands.print("The robot is aligning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

        // NamedCommands.registerCommand("scoreCoralL1",
        //     new SequentialCommandGroup(
        //         new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition).asProxy(),
        //         new AlignToReefTagRelative(m_SwerveSubsystem),
        //         new IntakeClosedLoop(m_IntakeSubsystem, 1, false)
        //     )
        // );

        // Autos
        autoChooser = new SendableChooser<Command>();
        //autoChooser.addOption("Do Nothing", new InstantCommand(() -> {System.out.println("hi");}));
        autoChooser.addOption("1PieceIPole", AutoBuilder.buildAuto("1PieceIPole"));
        autoChooser.addOption("2PieceIKPoles", AutoBuilder.buildAuto("2PieceIKPoles"));
        autoChooser.addOption("3PieceIKJPoles", AutoBuilder.buildAuto("3PieceIKJPoles"));
        autoChooser.addOption("1PieceHPole", AutoBuilder.buildAuto("1PieceHPole"));
        autoChooser.addOption("2PieceHGPoles", AutoBuilder.buildAuto("2PieceHGPoles"));
        autoChooser.addOption("1PieceL1Center", AutoBuilder.buildAuto("1PieceL1Center"));
        autoChooser.addOption("Test Auto", AutoBuilder.buildAuto("New Auto"));  
        autoChooser.addOption("3PieceIKJPolesLimelight", AutoBuilder.buildAuto("3PieceIKJPolesLimelight"));
        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
