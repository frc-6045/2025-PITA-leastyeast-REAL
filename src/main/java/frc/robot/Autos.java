package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeAutoTest;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem m_DriveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;
    //private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;
        m_DriveSubsystem = drive;
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("DoNothing", new InstantCommand(()->{System.out.println("hello I am doing nothing!");}));
        autoChooser.addOption("DoNothing2", new InstantCommand());
        autoChooser.addOption("new auto!", AutoBuilder.buildAuto("New Auto"));
        autoChooser.addOption("runIntake", new IntakeAutoTest(intake, 5000));
        SmartDashboard.putData("autos", autoChooser);
        //Shuffleboard.getTab("Test").add("test!!!",autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
