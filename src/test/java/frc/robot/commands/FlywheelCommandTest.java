package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FlywheelSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FlywheelCommandTest {
    private FlywheelSubsystem flywheel;
    private FlywheelCommand command;

    @BeforeEach
    void setUp() {
        // Initialize HAL for simulation
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();

        // Clear command scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        flywheel = new FlywheelSubsystem();
        command = new FlywheelCommand(flywheel);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        if (flywheel != null) {
            flywheel.close();
        }
        SimHooks.resumeTiming();
    }

    @Test
    void testCommandRequiresFlywheel() {
        // Verify the command requires the flywheel subsystem
        assertTrue(command.getRequirements().contains(flywheel),
            "Command should require the flywheel subsystem");
    }

    @Test
    void testIsFinishedReturnsFalse() {
        // The command should never finish on its own (runs while button held)
        assertFalse(command.isFinished(),
            "Command should not finish on its own");
    }

    @Test
    void testInitializeDoesNotThrow() {
        // Initialize should not throw any exceptions
        assertDoesNotThrow(() -> command.initialize(),
            "Initialize should not throw");
    }

    @Test
    void testExecuteRunsFlywheel() {
        command.initialize();
        command.execute();

        // After execute, the flywheel should have a target RPM set
        // (from dashboard default value)
        assertTrue(flywheel.getTargetRPM() > 0,
            "Execute should set a target RPM on the flywheel");
    }

    @Test
    void testEndStopsFlywheel() {
        // First execute to set a target
        command.initialize();
        command.execute();

        // Verify target was set
        assertTrue(flywheel.getTargetRPM() > 0, "Target should be set after execute");

        // End should stop the flywheel
        command.end(false);

        assertEquals(0, flywheel.getTargetRPM(),
            "End should stop the flywheel and reset target to 0");
    }

    @Test
    void testEndWhenInterrupted() {
        command.initialize();
        command.execute();

        // End with interrupted=true should also stop
        command.end(true);

        assertEquals(0, flywheel.getTargetRPM(),
            "End (interrupted) should also stop the flywheel");
    }

    @Test
    void testCommandLifecycle() {
        // Test the full command lifecycle manually
        // This tests our code without depending on WPILib scheduler internals

        // Initialize
        command.initialize();
        assertEquals(0, flywheel.getTargetRPM(), "Target should be 0 after initialize");

        // Execute multiple times
        command.execute();
        assertTrue(flywheel.getTargetRPM() > 0, "Target should be set after execute");
        double firstTarget = flywheel.getTargetRPM();

        command.execute();
        assertEquals(firstTarget, flywheel.getTargetRPM(), "Target should remain consistent");

        // Command should not finish on its own
        assertFalse(command.isFinished(), "Command should not finish");

        // End (simulating button release)
        command.end(false);
        assertEquals(0, flywheel.getTargetRPM(), "Target should be 0 after end");
    }

    @Test
    void testMultipleExecuteCycles() {
        command.initialize();

        // Run multiple execute cycles
        for (int i = 0; i < 10; i++) {
            command.execute();
        }

        // Command should still not be finished
        assertFalse(command.isFinished(), "Command should not finish after multiple executes");

        // Flywheel should still have target set
        assertTrue(flywheel.getTargetRPM() > 0, "Flywheel should maintain target");
    }
}
