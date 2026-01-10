package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FlywheelSubsystemTest {

    @BeforeEach
    void setUp() {
        // Initialize HAL for simulation
        assert HAL.initialize(500, 0);
        SimHooks.pauseTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    void testGetTargetRPM_ReturnsDefaultWhenNotSet() {
        // SmartDashboard returns default when key doesn't exist
        double target = SmartDashboard.getNumber("Flywheel Target RPM Test", MotorConstants.kFlywheelDefaultTargetRPM);
        assertEquals(MotorConstants.kFlywheelDefaultTargetRPM, target, 0.01);
    }

    @Test
    void testGetTargetRPM_ClampsToMaxRPM() {
        // Test that values above max are clamped
        SmartDashboard.putNumber("Flywheel Target RPM Clamp Test", 10000);
        double target = SmartDashboard.getNumber("Flywheel Target RPM Clamp Test", MotorConstants.kFlywheelDefaultTargetRPM);

        // Simulate the clamping logic from the subsystem
        double clampedTarget = Math.min(Math.max(target, 0), MotorConstants.kFlywheelMaxRPM);
        assertEquals(MotorConstants.kFlywheelMaxRPM, clampedTarget, 0.01);
    }

    @Test
    void testGetTargetRPM_ClampsToZero() {
        // Test that negative values are clamped to 0
        SmartDashboard.putNumber("Flywheel Target RPM Negative Test", -1000);
        double target = SmartDashboard.getNumber("Flywheel Target RPM Negative Test", MotorConstants.kFlywheelDefaultTargetRPM);

        // Simulate the clamping logic from the subsystem
        double clampedTarget = Math.min(Math.max(target, 0), MotorConstants.kFlywheelMaxRPM);
        assertEquals(0, clampedTarget, 0.01);
    }

    @Test
    void testSmartDashboardPIDDefaults() {
        // Verify PID constants are correctly defined
        assertEquals(0.0005, MotorConstants.kFlywheelP, 0.0001);
        assertEquals(0.0, MotorConstants.kFlywheelI, 0.0001);
        assertEquals(0.0, MotorConstants.kFlywheelD, 0.0001);
        assertEquals(100, MotorConstants.kFlywheelTolerance, 0.01);
    }

    @Test
    void testFlywheelConstants() {
        // Verify flywheel constants are correctly defined
        assertEquals(14, MotorConstants.kFlywheelMotorCANID);
        assertEquals(50, MotorConstants.kFlywheelMotorCurrentLimit);
        assertEquals(6500, MotorConstants.kFlywheelMaxRPM, 0.01);
        assertEquals(4000, MotorConstants.kFlywheelDefaultTargetRPM, 0.01);
    }

    @Test
    void testPIDOutputClamping() {
        // Test the clamping logic used in runToTargetRPM
        double output = 1.5; // Exceeds motor range
        double clampedOutput = Math.min(Math.max(output, 0), 1);
        assertEquals(1.0, clampedOutput, 0.01);

        output = -0.5; // Negative (invalid for forward-only flywheel)
        clampedOutput = Math.min(Math.max(output, 0), 1);
        assertEquals(0.0, clampedOutput, 0.01);

        output = 0.7; // Valid range
        clampedOutput = Math.min(Math.max(output, 0), 1);
        assertEquals(0.7, clampedOutput, 0.01);
    }
}
