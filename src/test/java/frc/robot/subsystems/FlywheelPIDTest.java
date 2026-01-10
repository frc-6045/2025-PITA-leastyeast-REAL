package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.MotorConstants;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the PID controller logic used in FlywheelSubsystem.
 * These tests verify the PID behavior without requiring hardware.
 */
class FlywheelPIDTest {

    private PIDController pidController;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        SimHooks.pauseTiming();

        // Create a PID controller with the same defaults as FlywheelSubsystem
        pidController = new PIDController(
            MotorConstants.kFlywheelP,
            MotorConstants.kFlywheelI,
            MotorConstants.kFlywheelD
        );
        pidController.setTolerance(MotorConstants.kFlywheelTolerance);
    }

    @AfterEach
    void tearDown() {
        pidController.close();
        SimHooks.resumeTiming();
    }

    @Test
    void testPIDCalculatesPositiveOutputWhenBelowSetpoint() {
        // When current RPM is below target, PID should output positive value
        double currentRPM = 3000;
        double targetRPM = 4000;

        double output = pidController.calculate(currentRPM, targetRPM);

        assertTrue(output > 0, "PID should output positive value when below setpoint");
    }

    @Test
    void testPIDCalculatesNegativeOutputWhenAboveSetpoint() {
        // When current RPM is above target, PID should output negative value
        // (which gets clamped to 0 in the actual subsystem)
        double currentRPM = 5000;
        double targetRPM = 4000;

        double output = pidController.calculate(currentRPM, targetRPM);

        assertTrue(output < 0, "PID should output negative value when above setpoint");
    }

    @Test
    void testPIDAtSetpointWhenWithinTolerance() {
        // When current RPM is within tolerance, atSetpoint should return true
        double targetRPM = 4000;
        double currentRPM = 4050; // Within 100 RPM tolerance

        pidController.calculate(currentRPM, targetRPM);

        assertTrue(pidController.atSetpoint(), "Should be at setpoint when within tolerance");
    }

    @Test
    void testPIDNotAtSetpointWhenOutsideTolerance() {
        // When current RPM is outside tolerance, atSetpoint should return false
        double targetRPM = 4000;
        double currentRPM = 3800; // Outside 100 RPM tolerance

        pidController.calculate(currentRPM, targetRPM);

        assertFalse(pidController.atSetpoint(), "Should not be at setpoint when outside tolerance");
    }

    @Test
    void testPIDReset() {
        // After reset, accumulated error should be cleared
        pidController.calculate(3000, 4000);
        pidController.calculate(3500, 4000);

        pidController.reset();

        // After reset, the controller should behave as if it's fresh
        // The output should be based only on the new error
        double output = pidController.calculate(3900, 4000);
        assertTrue(output > 0, "After reset, PID should calculate fresh output");
    }

    @Test
    void testPIDSetPID() {
        // Test that PID gains can be updated dynamically (for dashboard tuning)
        double newP = 0.001;
        double newI = 0.0001;
        double newD = 0.00001;

        pidController.setPID(newP, newI, newD);

        assertEquals(newP, pidController.getP(), 0.0001);
        assertEquals(newI, pidController.getI(), 0.0001);
        assertEquals(newD, pidController.getD(), 0.0001);
    }

    @Test
    void testPIDSetTolerance() {
        // Test that tolerance can be updated dynamically
        double newTolerance = 200;

        pidController.setTolerance(newTolerance);

        // Verify by checking atSetpoint behavior with the new tolerance
        double targetRPM = 4000;
        double currentRPM = 4150; // Would be outside 100 RPM but within 200 RPM

        pidController.calculate(currentRPM, targetRPM);

        assertTrue(pidController.atSetpoint(), "Should be at setpoint with updated tolerance");
    }

    @Test
    void testPIDOutputScalesWithError() {
        // Larger error should produce larger output
        double smallError = pidController.calculate(3900, 4000); // 100 RPM error
        pidController.reset();
        double largeError = pidController.calculate(3000, 4000); // 1000 RPM error

        assertTrue(Math.abs(largeError) > Math.abs(smallError),
            "Larger error should produce larger output magnitude");
    }

    @Test
    void testPIDZeroErrorZeroOutput() {
        // When at exactly the setpoint, output should be zero (with P-only control)
        double output = pidController.calculate(4000, 4000);

        assertEquals(0, output, 0.0001, "Zero error should produce zero output");
    }
}
