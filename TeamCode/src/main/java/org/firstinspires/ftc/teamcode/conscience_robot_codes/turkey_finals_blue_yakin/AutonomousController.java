package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals_blue_yakin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Improved Autonomous State Machine
 * Event-based, not time-based
 * Handles 3 shots with proper state transitions
 */

public class AutonomousController {
    private RobotHardware robot;
    private LinearOpMode opMode;

    // State machine for autonomous
    enum AutoState {
        SEARCH_TARGET,
        ALIGN_TO_TARGET,
        SPOOL_SHOOTER,
        DRIVE_TO_LINE,
        REVERSE_TO_LINE,
        LEFT_DRIVE_TO_LINE,
        FINISHED
    }

    private AutoState currentAutoState = AutoState.REVERSE_TO_LINE;
    private int shotsCompleted = 0;
    // private final int TOTAL_SHOTS = 3;

    // Timing for state transitions (safety fallbacks only)
    private ElapsedTime stateTimer;
    private final double STATE_TIMEOUT = 7.0; // 5 second timeout per state

    // Shot parameters
    private double targetVelocity = 0;
    private AprilTagDetection lastKnownTag = null;
    private boolean targetAligned = false;

    public AutonomousController(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
        this.stateTimer = new ElapsedTime();
    }

    /**
     * Main autonomous loop - call this in a while(opModeIsActive()) loop
     */
    public boolean executeAutoSequence() {
        switch (currentAutoState) {
            case SEARCH_TARGET:
                handleSearchTarget();
                break;
            case ALIGN_TO_TARGET:
                handleAlignToTarget();
                break;
            case SPOOL_SHOOTER:
                handleSpoolShooter();
                break;
            case DRIVE_TO_LINE:
                handleDriveToLine();
                break;
            case REVERSE_TO_LINE:
                handleReverseDriveToLine();
                break;
            case LEFT_DRIVE_TO_LINE:
                handleLeftDriveToLine();
                break;
            case FINISHED:
                return false; // Signal completion
        }
        return true; // Continue loop
    }

    private void handleReverseDriveToLine() {
        // Non-blocking drive setup
        if (stateTimer.milliseconds() < 100) {
            // First iteration - set up the drive
            robot.autoDrive(0.7, -40, -40, 3000);
            opMode.telemetry.addData("Auto", "Driving to line...");
        }

        // Move to finished state
        currentAutoState = AutoState.SEARCH_TARGET;
    }

    private void handleSearchTarget() {
        AprilTagDetection tag = robot.getLatestTargetDetection();

        if (tag != null) {
            lastKnownTag = tag;
            stateTimer.reset();
            currentAutoState = AutoState.ALIGN_TO_TARGET;
            opMode.telemetry.addData("Auto", "Target found! Aligning...");
        } else {
            if (stateTimer.seconds() > STATE_TIMEOUT) {
                // Timeout - proceed with last known velocity or bank shot
                targetVelocity = robot.bankVelocity;
                currentAutoState = AutoState.SPOOL_SHOOTER;
                stateTimer.reset();
            }
            opMode.telemetry.addData("Auto", "Searching for target...");
        }
    }

    private void handleAlignToTarget() {
        // Refresh tag data every cycle
        AprilTagDetection tag = robot.getLatestTargetDetection();

        if (tag != null) {
            lastKnownTag = tag;
            boolean aligned = robot.updateAngularAlignment(tag);

            opMode.telemetry.addData("Alignment", "Error: %.1f°", tag.ftcPose.bearing);
            opMode.telemetry.addData("Range", "%.1f in", tag.ftcPose.range);

            if (aligned) {
                // Successfully aligned!
                robot.stopDrive();
                robot.resetIntegralSum();
                targetVelocity = robot.getCalculatedShotVelocity();
                stateTimer.reset();
                currentAutoState = AutoState.SPOOL_SHOOTER;
                opMode.telemetry.addData("Auto", "Aligned! Target Velocity: %.0f", targetVelocity);
            }
        } else {
            // Lost target mid-alignment
            opMode.telemetry.addData("Auto", "TARGET LOST!");
            robot.stopDrive();
            robot.resetIntegralSum();

            if (stateTimer.seconds() > STATE_TIMEOUT) {
                // Timeout - proceed with last known velocity or bank shot
                if (lastKnownTag != null) {
                    targetVelocity = robot.getCalculatedShotVelocity();
                    currentAutoState = AutoState.SPOOL_SHOOTER;
                    stateTimer.reset();
                } else {
                    // Blind bank shot
                    targetVelocity = robot.bankVelocity;
                    currentAutoState = AutoState.SPOOL_SHOOTER;
                    stateTimer.reset();
                }
            }
        }
    }

    private void handleSpoolShooter() {
        robot.setShooterVelocity(targetVelocity);
        double currentVel = robot.flywheel.getVelocity();
        double targetVel = targetVelocity;

        // Check if shooter is up to speed (within 50 ticks/sec)
        if (currentVel >= targetVel - 5) {
            robot.setFeederPower(1.0);
            opMode.sleep(250);
            robot.setFeederPower(0);
            // stateTimer.reset();
            // currentAutoState = AutoState.FIRE_SHOT;
            // opMode.telemetry.addData("Auto", "Shooter ready! Firing...");
        } else {
            opMode.telemetry.addData("Auto", "Spooling... %.0f / %.0f", currentVel, targetVel);
        }

        // Safety timeout
        if (stateTimer.seconds() > STATE_TIMEOUT) {
            // opMode.telemetry.addData("Auto", "SPOOL TIMEOUT - Firing anyway");
            currentAutoState = AutoState.LEFT_DRIVE_TO_LINE;
            stateTimer.reset();
        }
    }

    private void handleLeftDriveToLine() {
        // Non-blocking drive setup
        if (stateTimer.milliseconds() < 100) {
            // First iteration - set up the drive
            robot.autoDrive(0.7, -6, 6, 1000);
            opMode.telemetry.addData("Auto", "Driving to line...");
        }

        // Move to finished state
        currentAutoState = AutoState.DRIVE_TO_LINE;
    }

    private void handleDriveToLine() {
        // Non-blocking drive setup
        if (stateTimer.milliseconds() < 100) {
            // First iteration - set up the drive
            robot.autoDrive(0.7, 40, 40, 3000);
            opMode.telemetry.addData("Auto", "Driving to line...");
        }

        // Move to finished state
        currentAutoState = AutoState.FINISHED;
    }


    public String getCurrentState() {
        return currentAutoState.toString();
    }

    public int getShotsCompleted() {
        return shotsCompleted;
    }
}
