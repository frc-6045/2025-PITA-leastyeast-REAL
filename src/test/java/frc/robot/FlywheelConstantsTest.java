package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.FlywheelConstants;

import org.junit.jupiter.api.Test;

class FlywheelConstantsTest {

    @Test
    void testCANIDIsValid() {
        // CAN ID should be between 1 and 62 (valid range for CAN bus)
        assertTrue(FlywheelConstants.kFlywheelMotorCANID >= 1,
            "CAN ID should be at least 1");
        assertTrue(FlywheelConstants.kFlywheelMotorCANID <= 62,
            "CAN ID should be at most 62");
    }

    @Test
    void testCANIDIsUnique() {
        // Verify CAN ID 14 doesn't conflict with other motor CAN IDs
        int flywheelCANID = FlywheelConstants.kFlywheelMotorCANID;

        assertNotEquals(Constants.MotorConstants.kArmMotorCANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Arm motor");
        assertNotEquals(Constants.MotorConstants.kElevatorMotor1CANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Elevator motor 1");
        assertNotEquals(Constants.MotorConstants.kElevatorMotor2CANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Elevator motor 2");
        assertNotEquals(Constants.MotorConstants.kIntakeMotorCANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Intake motor");
        assertNotEquals(Constants.MotorConstants.kClimbMotorCANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Climb motor");
        assertNotEquals(Constants.MotorConstants.kClimbWristMotorCANID, flywheelCANID,
            "Flywheel CAN ID should not conflict with Climb Wrist motor");
    }

    @Test
    void testCurrentLimitIsReasonable() {
        // Current limit should be positive and within typical range (20-80A for NEO/Vortex)
        assertTrue(FlywheelConstants.kFlywheelMotorCurrentLimit > 0,
            "Current limit should be positive");
        assertTrue(FlywheelConstants.kFlywheelMotorCurrentLimit <= 80,
            "Current limit should not exceed 80A");
    }

    @Test
    void testPIDGainsAreNonNegative() {
        assertTrue(FlywheelConstants.kDefaultP >= 0,
            "P gain should be non-negative");
        assertTrue(FlywheelConstants.kDefaultI >= 0,
            "I gain should be non-negative");
        assertTrue(FlywheelConstants.kDefaultD >= 0,
            "D gain should be non-negative");
    }

    @Test
    void testFeedforwardIsReasonable() {
        // Feedforward should be positive and small (typically 1/maxRPM range)
        assertTrue(FlywheelConstants.kDefaultFF > 0,
            "Feedforward should be positive");
        assertTrue(FlywheelConstants.kDefaultFF < 0.01,
            "Feedforward should be small (< 0.01)");
    }

    @Test
    void testTargetRPMIsWithinMax() {
        assertTrue(FlywheelConstants.kDefaultTargetRPM > 0,
            "Default target RPM should be positive");
        assertTrue(FlywheelConstants.kDefaultTargetRPM <= FlywheelConstants.kMaxRPM,
            "Default target RPM should not exceed max RPM");
    }

    @Test
    void testMaxRPMIsReasonable() {
        // Max RPM for NEO Vortex is around 6500
        assertTrue(FlywheelConstants.kMaxRPM > 0,
            "Max RPM should be positive");
        assertTrue(FlywheelConstants.kMaxRPM <= 10000,
            "Max RPM should be within reasonable motor limits");
    }

    @Test
    void testToleranceIsPositive() {
        assertTrue(FlywheelConstants.kDefaultToleranceRPM > 0,
            "Tolerance should be positive");
    }

    @Test
    void testToleranceIsReasonable() {
        // Tolerance should be small enough to be useful but not too tight
        assertTrue(FlywheelConstants.kDefaultToleranceRPM >= 10,
            "Tolerance should be at least 10 RPM");
        assertTrue(FlywheelConstants.kDefaultToleranceRPM <= 500,
            "Tolerance should not exceed 500 RPM");
    }

    @Test
    void testFeedforwardCalculation() {
        // Verify the feedforward is roughly 1/maxRPM order of magnitude
        // This ensures the FF term produces reasonable output
        double expectedFFRange = 1.0 / FlywheelConstants.kMaxRPM;

        // FF should be within an order of magnitude of 1/maxRPM
        assertTrue(FlywheelConstants.kDefaultFF >= expectedFFRange * 0.1,
            "Feedforward should be at least 10% of 1/maxRPM");
        assertTrue(FlywheelConstants.kDefaultFF <= expectedFFRange * 10,
            "Feedforward should be at most 10x of 1/maxRPM");
    }
}
