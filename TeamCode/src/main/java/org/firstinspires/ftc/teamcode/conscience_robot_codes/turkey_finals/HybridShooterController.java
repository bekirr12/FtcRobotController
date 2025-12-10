package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals;

/**
 * Hybrid Shooter Controller
 * - Polynomial Velocity Control (Precise & Smooth)
 * - Fixed angle system (angle control removed)
 * Calibration Required: 8-10 distance points
 */
public class HybridShooterController {

    // VELOCITY POLYNOMIAL COEFFICIENTS
    // velocity = v0 + v1*r + v2*r² + v3*r³
    private double v0 = 1200.0;     // Baseline velocity
    private double v1 = 15.0;       // Linear term
    private double v2 = -0.15;      // Quadratic term
    private double v3 = 0.002;      // Cubic term

    // Velocity limits
    private final int MIN_VELOCITY = 1000;
    private final int MAX_VELOCITY = 2400;

    /* ANGLE CONTROL REMOVED - FIXED ANGLE SYSTEM
    // ANGLE POLYNOMIAL COEFFICIENTS
    // angle = a0 + a1*r + a2*r² + a3*r³
    private double a0 = 0.35;      // Baseline angle
    private double a1 = 0.0098;    // Linear term
    private double a2 = -0.00018;  // Quadratic term
    private double a3 = 0.0000015; // Cubic term

    // Servo limits
    private final double MIN_ANGLE = 0.15;
    private final double MAX_ANGLE = 0.85;
    */

    // MAIN CALCULATION METHOD

    /**
     * Calculate velocity based on distance (polynomial)
     */
    public int calculateVelocity(double distance) {
        // Input validation
        distance = Math.max(15.0, Math.min(65.0, distance));

        // Polynomial evaluation
        double velocity = v0 +
                v1 * distance +
                v2 * distance * distance +
                v3 * distance * distance * distance;

        // Velocity safety limits
        return (int) Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, velocity));
    }

    /* ANGLE CALCULATION REMOVED - FIXED ANGLE SYSTEM
    /**
     * calculate angle depends on distance (polynomial)
     */
    /*
    public double calculateAngle(double distance) {
        // Input validation
        distance = Math.max(15.0, Math.min(65.0, distance));

        // Polynomial evaluation
        double angle = a0 +
                a1 * distance +
                a2 * distance * distance +
                a3 * distance * distance * distance;

        // Servo safety limits
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }
    */
}