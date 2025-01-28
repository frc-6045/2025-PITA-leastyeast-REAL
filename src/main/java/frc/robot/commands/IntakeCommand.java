package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final CommandXboxController controller;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, CommandXboxController xboxController) {
        m_IntakeSubsystem = intakeSubsystem;
        controller = xboxController;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = MotorConstants.kIntakeMotorsSpeed;
        double triggerAxis = controller.getLeftTriggerAxis()-controller.getRightTriggerAxis();
        m_IntakeSubsystem.setSpeed(speed, triggerAxis, controller.getRightX() > .10 ? 0.0: triggerAxis); //Grant's Ternary. Press right on stick and it won't run second rollers. Should be a button but I couldn't figure out how to return bool from a button if its pressed or not :)
        System.out.println("INTAKE speed: " + speed + "\nbumperPressed: " + triggerAxis + "\ntriggerAxis: " + triggerAxis);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("BUMPERPRESSED TO 0");
        m_IntakeSubsystem.setSpeed(0, 0, 0);    
    }

}
