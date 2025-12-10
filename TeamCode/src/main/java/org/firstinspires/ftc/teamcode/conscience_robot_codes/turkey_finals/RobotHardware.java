package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
* Hardware Abstraction Layer for FTC Robot
* Handles all sensor fusion, PID control, and hardware interfacing

* Turkey Finals 2025
*/

public class RobotHardware {
    private LinearOpMode myOpMode = null;

    // hardware objects
    public DcMotorEx flywheel = null;
    public DcMotor coreHex = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public CRServo hopperServo = null;
    public Servo gateServo = null;
    // public Servo turretServo = null; // Removed adjustable shooter
    public IMU imu = null;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private HybridShooterController shooterController;

    // manuel velocity constants
    public final int bankVelocity = 1350;
    public final int farVelocity = 1550;
    public final int maxVelocity = 2200;

    // Drive encoder conversion constant
    public final double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);

    public final int TARGET_TAG_ID = 20; // for blue goal (change to 24 for red)

    /*
    // Linear interpolation constants (Buraları robotun üstünde test et düzenle)
    public final double DIST_1_CLOSE = 15.0;
    public final int    VEL_1_CLOSE  = 1450;
    public final double DIST_2_MID   = 30.0;
    public final int    VEL_2_MID    = 1750;
    public final double DIST_3_FAR   = 45.0;
    public final int    VEL_3_FAR    = 2100;
    public final double DIST_4_MAX   = 60.0;
    public final int    VEL_4_MAX    = 2400;
    */

    // PID-Controller Gains (Tune these to fit your robot)
    private final double TURN_GAIN = 0.03; // P-Gain (Kp)
    private final double DERIVATIVE_GAIN = 0.015;    // D-Gain (Kd)
    private final double INTEGRAL_GAIN = 0.005; // I-Gain (Ki)
    private final double MAX_AUTO_TURN = 0.4;
    private final double INTEGRAL_MAX = 5.0;
    public final double ALIGN_HEADING_TOLERANCE = 1.0;
    private final double YAW_RATE_STOP_THRESHOLD = 5.0;

    /* Removed adjustable shooter
    // Launcher control
    private final double MAX_TURRET_RATE = 0.10;  // Max servo değişim hızı (per 20ms cycle)
    private final double TURRET_SETTLING_TIME_MS = 150;  // Servo oturma süresi
    private double lastTurretPosition = 0.5;  // Son komut edilen pozisyon
    private ElapsedTime turretTimer;
     */

    private double integralSum = 0;
    public double calculatedShotVelocity = 0;
    // public double calculatedTurretAngle = 0; // Removed adjustable shooter
    private double speedMultiplier = 0.5;
    private int calibrationVelocity = 1000;

    // timers
    private ElapsedTime autoDriveTimer;
    public ElapsedTime autoLaunchTimer;

    // initialization method
    public void init(LinearOpMode opMode, HardwareMap hwMap) {
        this.myOpMode = opMode;

        // initialize hardware
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        coreHex = hwMap.get(DcMotor.class, "coreHex");
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        hopperServo = hwMap.get(CRServo.class, "hopperServo");
        gateServo = hwMap.get(Servo.class, "gateServo");
        // turretServo = hwMap.get(Servo.class, "turretServo"); // Removed adjustable shooter
        imu = hwMap.get(IMU.class, "imu");

        initIMU(hwMap);
        initAprilTag(hwMap);

        shooterController = new HybridShooterController();

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hopperServo.setPower(0);
        gateServo.setPosition(0);

        /* // Removed adjustable shooter
        turretServo.setPosition(0.5);
        lastTurretPosition = 0.5;
        turretTimer = new ElapsedTime();
         */

        autoLaunchTimer = new ElapsedTime();
        autoDriveTimer = new ElapsedTime();

    }

    /**
     * Configures the IMU with the correct orientation
     */
    private void initIMU(HardwareMap hwMap) {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    /**
     * Configures the AprilTag vision processor
     */
    private void initAprilTag(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    // GETTERS (Encapsulation)
    public double getCalculatedShotVelocity() {
        return calculatedShotVelocity;
    }
    /* // Removed adjustable shooter
    public double getCalculatedTurretAngle() {
        return calculatedTurretAngle;
    }
     */
    public void resetIntegralSum() {
        integralSum = 0;
    }

    /** Gets the latest detection for the target tag. Returns null if not seen. */
    public AprilTagDetection getLatestTargetDetection() {
        if (visionPortal != null && aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                    return detection;
                }
            }
        }
        return null;
    }

    /* Calculates the ideal flywheel velocity using 4-point interpolation based on distance.
    public int calculateTargetVelocity(double distance) {
        if (distance <= DIST_1_CLOSE) return VEL_1_CLOSE;
        if (distance >= DIST_4_MAX)   return VEL_4_MAX;

        double x1, x2;
        int y1, y2;

        if (distance < DIST_2_MID) {
            x1=DIST_1_CLOSE; y1=VEL_1_CLOSE;
            x2=DIST_2_MID; y2=VEL_2_MID;
        } else if (distance < DIST_3_FAR) {
            x1=DIST_2_MID; y1=VEL_2_MID;
            x2=DIST_3_FAR; y2=VEL_3_FAR;
        } else {
            x1=DIST_3_FAR; y1=VEL_3_FAR;
            x2=DIST_4_MAX; y2=VEL_4_MAX;
        }
        return (int) (y1 + (distance - x1) * (y2 - y1) / (x2 - x1));
    }
    */

    /**
    Hybrid System: PID alignment + turret angle control
     */

    /**
     * PID-controlled angular alignment to AprilTag target
     * Uses P-term for correction, D-term for damping, I-term for steady-state
     * */
    public boolean updateAngularAlignment(AprilTagDetection target) {
        /*
        AprilTagDetection target = getLatestTargetDetection();

        if (target == null) {
            stopDrive();
            resetIntegralSum();
            return false;
        }

         */

        double headingError = target.ftcPose.bearing;
        double dist = target.ftcPose.range;

        // calculatedShotVelocity = calculateTargetVelocity(dist);
        calculatedShotVelocity = shooterController.calculateVelocity(dist); // new
        // calculatedTurretAngle = shooterController.calculateAngle(dist);  // Removed adjustable shooter   // new

        // boolean turretReady = updateTurretPosition(calculatedTurretAngle);

        // D-Term: Braking force from IMU Yaw Rate
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double yawRate = angularVelocity.zRotationRate;

        // P-Term: Proportional correction
        double p_term = headingError * TURN_GAIN;

        // D-Term: Derivative correction
        double d_term = -yawRate * DERIVATIVE_GAIN;

        // I-Term: Integral correction
        integralSum += headingError;
        integralSum = Range.clip(integralSum, -INTEGRAL_MAX, INTEGRAL_MAX);
        double i_term = integralSum * INTEGRAL_GAIN;

        double totalTurnPower = p_term + i_term + d_term;
        double clippedTurnPower = Range.clip(totalTurnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        // Apply Power
        leftDrive.setPower(clippedTurnPower);
        rightDrive.setPower(-clippedTurnPower);

        // Stop Condition (Aligned AND Stopped)
        boolean angularAligned = Math.abs(headingError) < ALIGN_HEADING_TOLERANCE &&
                Math.abs(yawRate) < YAW_RATE_STOP_THRESHOLD;

        if (angularAligned && calculatedShotVelocity > 0) {
            stopDrive();
            resetIntegralSum();
            return true;
        }
        return false;
    }

    /* Removed adjustable shooter
    private boolean updateTurretPosition(double targetPosition) {
        double delta = targetPosition - lastTurretPosition;
        if (Math.abs(delta) > MAX_TURRET_RATE) {
            delta = Math.copySign(MAX_TURRET_RATE, delta);
            targetPosition = lastTurretPosition + delta;
        }

        turretServo.setPosition(targetPosition);
        lastTurretPosition = targetPosition;

        return Math.abs(delta) < 0.01;
    }
     */

    /* Removed adjustable shooter
    public void manualTurretControl(boolean dpadUp, boolean dpadDown) {
        if (turretServo == null) return;

        final double MANUAL_INCREMENT = 0.01;

        if (dpadUp) {
            lastTurretPosition = Math.min(0.85, lastTurretPosition + MANUAL_INCREMENT);
        } else if (dpadDown) {
            lastTurretPosition = Math.max(0.15, lastTurretPosition - MANUAL_INCREMENT);
        }

        turretServo.setPosition(lastTurretPosition);
    }
     */

    /* Removed adjustable shooter
    public void resetTurret() {
        if (turretServo != null) {
            turretServo.setPosition(0.5);
            lastTurretPosition = 0.5;
        }
    }
     */

    public int manualVelocityControl(boolean dpadLeft, boolean dpadRight) {
        final int MANUAL_INCREMENT_VELOCITY = 50;

        if (dpadLeft) {
            calibrationVelocity = Math.max(0, calibrationVelocity - MANUAL_INCREMENT_VELOCITY);
        } else if (dpadRight) {
            calibrationVelocity = Math.min(2400, calibrationVelocity + MANUAL_INCREMENT_VELOCITY);
        }

        setShooterVelocity(calibrationVelocity);
        return calibrationVelocity;
    }

    // Shooter Control Methods
    public void setShooterVelocity(double velocity) {
        flywheel.setVelocity(velocity);
    }

    public void setFeederPower(double power) {
        coreHex.setPower(power);
        hopperServo.setPower(power);
    }

    public void stopDrive() {
        leftDrive.setPower(0); rightDrive.setPower(0);
    }

    /** Sets the launcher for close shooting and activates the feeder. */
    public void bankShotAuto() {
        flywheel.setVelocity(bankVelocity);
        hopperServo.setPower(-1);
        // turretServo.setPosition(0.5); // test and fix
        if (flywheel.getVelocity() >= bankVelocity - 50) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    /** Sets the launcher for long throw and activates the feeder. */
    public void farPowerAuto() {
        flywheel.setVelocity(farVelocity);
        hopperServo.setPower(-1);
        // turretServo.setPosition(0.5); // test and fix
        if (flywheel.getVelocity() >= farVelocity - 100) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    /** Implements split-stick arcade driving using gamepad data. */
    public void splitStickArcadeDrive(double x_stick, double y_stick) {
        double x = x_stick * speedMultiplier;
        double y = y_stick * speedMultiplier;

        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    /** Updates the robot's speed multiplier. */
    public void updateSpeedMultiplier(boolean dpadUpPressed, boolean dpadDownPressed) {
        if (dpadUpPressed) {
            speedMultiplier = Math.min(speedMultiplier + 0.1, 1.0);
        } else if (dpadDownPressed) {
            speedMultiplier = Math.max(speedMultiplier - 0.1, 0.1);
        }
    }

    /** Controls the Core Hex feeder and CRServo based on button states. */
    public void manualServoAndCoreHexControl(boolean cross, boolean triangle, boolean dpadLeft, boolean dpadRight) {
        // Manual control for the Core Hex feeder
        if (cross) {
            coreHex.setPower(0.5);
        } else if (triangle) {
            coreHex.setPower(-0.5);
        } else {
            coreHex.setPower(0);
        }

        // Manual control for the hopper's servo (CRServo)
        if (dpadLeft) {
            hopperServo.setPower(1);
        } else if (dpadRight) {
            hopperServo.setPower(-1);
        } else {
            hopperServo.setPower(0);
        }
    }

    /** Controls the position-based Servo Gate (servo2). */
    public void smartServoGate(boolean dpadUp, boolean dpadDown) {
        if (dpadUp) {
            gateServo.setPosition(0.5);
        } else if (dpadDown) {
            gateServo.setPosition(0);
        }
    }

    /** Controls the flywheel velocity based on multiple button states. */
    public void setFlywheelVelocity(boolean options, boolean leftBumper, boolean rightBumper, boolean circle, boolean square, boolean dpadRight, boolean dpadLeft) {
        if (options) {
            flywheel.setPower(-0.5);
        } else if (leftBumper) {
            farPowerAuto();
        } else if (rightBumper) {
            bankShotAuto();
        } else if (circle) {
            flywheel.setVelocity(bankVelocity);
        } else if (square) {
            flywheel.setVelocity(maxVelocity);
        } else {
            flywheel.setVelocity(0);
            coreHex.setPower(0);

            // Stop the Continuous Servo only if D-pad isn't being pressed
            if (!dpadRight && !dpadLeft) {
                hopperServo.setPower(0);
            }
        }
    }

    /** Drives the robot to the specified distance using encoders (blocking call). */
    public void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        autoDriveTimer.reset();

        double WHEELS_INCHES_TO_TICKS_LOCAL = WHEELS_INCHES_TO_TICKS;

        leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS_LOCAL));
        rightDrive.setTargetPosition((int) (rightDrive.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS_LOCAL));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {
            myOpMode.idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}