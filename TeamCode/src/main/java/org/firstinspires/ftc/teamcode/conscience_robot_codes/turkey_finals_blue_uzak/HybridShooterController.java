package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals_blue_uzak;

/**
 * Hybrid Shooter Controller
 * - Polynomial Velocity Control (Precise & Smooth)
 * - Fixed angle system (angle control removed)
 * Calibration Required: 8-10 distance points
 */
public class HybridShooterController {

    // VELOCITY POLYNOMIAL COEFFICIENTS
    // velocity = v0 + v1*r + v2*r² + v3*r³
    /*
    private double v0 = 3880.158707;     // Baseline velocity
    private double v1 = -70.393888;       // Linear term
    private double v2 = 0.697250;      // Quadratic term
    private double v3 = -0.002084;      // Cubic term
    */
    private double v0 = 3450.025958;     // Baseline velocity
    private double v1 = -57.105962;       // Linear term
    private double v2 = 0.574475;      // Quadratic term
    private double v3 = -0.001769;      // Cubic term


    // Velocity limits
    private final int MIN_VELOCITY = 1000;
    private final int MAX_VELOCITY = 2400;


    // MAIN CALCULATION METHOD

    /**
     * Calculate velocity based on distance (polynomial)
     */
    public int calculateVelocity(double distance) {
        // Input validation
        distance = Math.max(30.0, Math.min(150.0, distance));

        // Polynomial evaluation
        double velocity = v0 +
                v1 * distance +
                v2 * distance * distance +
                v3 * distance * distance * distance;

        // Velocity safety limits
        return (int) Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, velocity));
    }

}