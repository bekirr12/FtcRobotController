package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals;

/**
 * Hybrid Shooter Controller
 * - 3-Zone Velocity System (Simple & Robust)
 * - Polynomial Angle Control (Precise & Smooth)
 * Calibration Required: 8-10 distance points
 */
public class HybridShooterController {

    // VELOCITY ZONES (3-Zone System)
    private final double ZONE_1_MAX = 28.0;  // inch - close/medium line
    private final double ZONE_2_MAX = 42.0;  // inch - medium/far line

    private final int VELOCITY_CLOSE = 1550;  // close shot velocity
    private final int VELOCITY_MID = 1800;    // medium shot velocity
    private final int VELOCITY_FAR = 2050;    // far shot velocity

    // ANGLE POLYNOMIAL COEFFICIENTS
    // angle = a0 + a1*r + a2*r² + a3*r³
    private double a0 = 0.35;      // Baseline angle
    private double a1 = 0.0098;    // Linear term
    private double a2 = -0.00018;  // Quadratic term
    private double a3 = 0.0000015; // Cubic term

    // Servo limits
    private final double MIN_ANGLE = 0.15;
    private final double MAX_ANGLE = 0.85;

    // MAIN CALCULATION METHODS

    /**
     * calculate velocity depends on distance
     */
    public int calculateVelocity(double distance) {
        if (distance < ZONE_1_MAX) {
            return VELOCITY_CLOSE;
        } else if (distance < ZONE_2_MAX) {
            return VELOCITY_MID;
        } else {
            return VELOCITY_FAR;
        }
    }

    /**
     * calculate angle depends on distance (polynomial)
     */
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
}
