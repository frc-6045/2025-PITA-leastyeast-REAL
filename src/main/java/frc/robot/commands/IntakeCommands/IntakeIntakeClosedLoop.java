package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeSubsystem;

/** Either controller triggers move intake. */
public class IntakeIntakeClosedLoop extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final BooleanSupplier coralDetected;
    private final Timer timer = new Timer();
    private final Timer coralTimer = new Timer();
    
    public IntakeIntakeClosedLoop(IntakeSubsystem intakeSubsystem, BooleanSupplier coralDetected) {
        m_IntakeSubsystem = intakeSubsystem;
        this.coralDetected = coralDetected;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        coralTimer.reset();
    }

    @Override
    public void execute() {
        double triggerAxis = -MotorConstants.kIntakeMotorMaxSpeed;
        m_IntakeSubsystem.setSpeed(triggerAxis);
        if (coralDetected.getAsBoolean() && coralTimer.get() == 0) {
            coralTimer.start();
            System.out.println("coraltimer started");
        }
    }

    @Override
    public boolean isFinished() {
        if (coralTimer.get() > 0.13 || timer.get() > 2) {
            System.out.println("intakeintake end :3");
            coralTimer.stop();
            coralTimer.reset();
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();   
    }

}
