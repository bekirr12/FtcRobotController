package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 2025 FTC DECODE sezonunda Türkiye Finalleri için kullanilacak kod.

 FTC Yazilim Ekibi
 Bekir Sami Karatas
 22.11.2025
 */
@TeleOp(name = "TR_Finalleri_Kodu", group = "Competition")
public class Auto_and_TeleOp extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();

    // --- STATE MACHINE DEFINITIONS ---
    enum RobotState {
        MANUAL_DRIVE,
        AUTO_ALIGN,
        SHOOTING,
        RESET
    }
    RobotState currentState = RobotState.MANUAL_DRIVE;

    // --- OPMODE SELECTION CONSTANTS ---
    private final String AUTO_BLUE = "AUTO BLUE";
    private final String AUTO_RED = "AUTO RED";
    private final String TELEOP = "TELEOP";

    private String operationSelected = TELEOP;
    private double activeShotVelocity = 0;

    // MAIN EXECUTION FLOW (runOpMode)
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, hardwareMap);
        telemetry.addData("Status", "Sniper System Initialized");
        telemetry.addData("Info", "Press PS to cycle modes");

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.psWasPressed()) {
                operationSelected = selectOperation(operationSelected);
            }
            updateSelectionTelemetry();
            idle();
        }

        waitForStart();

        if (opModeIsActive()) {
            if (operationSelected.equals(AUTO_BLUE)) {
                doAutoBlue();
            } else if (operationSelected.equals(AUTO_RED)) {
                doAutoRed();
            } else {
                doTeleOpStateful();
            }
        }
    }

    // TELEOP STATE MACHINE EXECUTION
    private void doTeleOpStateful() {
        while (opModeIsActive()) {

            switch (currentState) {
                case MANUAL_DRIVE:
                    handleManualState();
                    break;
                case AUTO_ALIGN:
                    handleAutoAlignState();
                    break;
                case SHOOTING:
                    handleShootingState();
                    break;
                case RESET:
                    handleResetState();
                    break;
            }
            telemetry.addData("Current State", currentState);
            telemetry.addData("Flywheel Vel", robot.flywheel.getVelocity());
            telemetry.addData("left power", robot.leftDrive.getPower());
            telemetry.addData("right power", robot.rightDrive.getPower());
            telemetry.addData("hex motor power", robot.coreHex.getPower());
            telemetry.update();
        }
    }

    // STATE HANDLER METHODS

    private void handleManualState() {
        double x_s = gamepad2.left_stick_x;
        double y_s  = -gamepad2.left_stick_y;
        robot.splitStickArcadeDrive(x_s, y_s); // Uses speedMultiplier internally
        manualMechanismControlLogic();

        if (gamepad1.right_trigger > 0.5) {
            currentState = RobotState.AUTO_ALIGN;
        }
    }

    private void handleAutoAlignState() {
        AprilTagDetection tag = robot.getLatestTargetDetection();
        boolean aligned = false;

        if (gamepad1.right_trigger < 0.1) {
            currentState = RobotState.RESET;
            return;
        }

        if (tag != null) {
            aligned = robot.updateAngularAlignment(tag);
            telemetry.addData("Aiming", "Error: %.1f° | Range: %.1f in",
                    tag.ftcPose.bearing, tag.ftcPose.range);
        } else {
            gamepad2.rumble(1.0, 1.0, 200);
            telemetry.addData("Aiming", "TARGET LOST");
            currentState = RobotState.RESET;
            // robot.stopDrive();
            // robot.resetIntegralSum();
        }

        if (aligned && robot.getCalculatedShotVelocity() > 0) {
            activeShotVelocity = robot.getCalculatedShotVelocity();
            robot.stopDrive();
            robot.setShooterVelocity(activeShotVelocity);
            currentState = RobotState.SHOOTING;
        }
    }

    private void handleShootingState() {
        if (gamepad1.right_trigger < 0.1) {
            currentState = RobotState.RESET;
            return;
        }

        if (robot.flywheel.getVelocity() >= activeShotVelocity - 50) {
            robot.setFeederPower(1.0);
            telemetry.addData("Shot Status", "FIRING %.0f ticks/sec", activeShotVelocity);
        } else {
            telemetry.addData("Shot Status", "Spooling... %.0f / %.0f",
                    robot.flywheel.getVelocity(), activeShotVelocity);
        }
    }

    private void handleResetState() {
        robot.setFeederPower(0);
        robot.setShooterVelocity(0);
        robot.stopDrive();
        robot.resetIntegralSum();
        currentState = RobotState.MANUAL_DRIVE;
    }

    // Manuel control helper function
    private void manualMechanismControlLogic() {
        robot.updateSpeedMultiplier(gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());

        robot.setFlywheelVelocity(
                gamepad1.options, gamepad1.left_bumper, gamepad1.right_bumper,
                gamepad1.circle, gamepad1.square, gamepad1.dpad_right, gamepad1.dpad_left
        );

        robot.manualServoAndCoreHexControl(
                gamepad1.cross, gamepad1.triangle, gamepad1.dpad_left, gamepad1.dpad_right
        );

        robot.smartServoGate(gamepad1.dpad_up, gamepad1.dpad_down);
    }

    private String selectOperation(String state) {
        switch (state) {
            case "TELEOP": state = AUTO_BLUE; break;
            case "AUTO BLUE": state = AUTO_RED; break;
            case "AUTO RED": state = TELEOP; break;
            default: state = TELEOP; break;
        }
        return state;
    }

    private void updateSelectionTelemetry() {
        telemetry.addLine("Press PS Button to Cycle Options");
        telemetry.addData("ACTIVE SELECTION", operationSelected);
        telemetry.update();
    }

    private void doAutoBlue() throws InterruptedException {
        executeSmartAutoShot();
        robot.autoDrive(0.7, 37, 37, 2000);
        // robot.autoDrive(0.5, -11, 7, 5000);
        // robot.autoDrive(0.5, 50, 50, 5000);
    }
    private void doAutoRed() throws InterruptedException {
        executeSmartAutoShot();
        robot.autoDrive(0.7, 37, 37, 2000);
        // robot.autoDrive(0.5, 12, -7, 5000);
        // robot.autoDrive(0.5, 50, 50, 5000);
    }

    /** Executes the PD-Controlled Angular Alignment and fires the calculated velocity shot. */
    private void executeSmartAutoShot() throws InterruptedException {
        AprilTagDetection tag = robot.getLatestTargetDetection();

        if (tag != null) {
            double dist = tag.ftcPose.range;
            double calculatedVel = robot.calculateTargetVelocity(dist);

            telemetry.addData("Auto Status", "Aligning to target...");
            telemetry.addData("Range", "%.1f inches", dist);
            telemetry.addData("Calc Velocity", "%.0f ticks/sec", calculatedVel);
            telemetry.update();

            // Wait for PID alignment to complete
            while (opModeIsActive() && !robot.updateAngularAlignment(tag)) {
                idle();
            }

            if (robot.updateAngularAlignment(tag) && calculatedVel > 0) {
                robot.setShooterVelocity(calculatedVel);

                // ElapsedTime spoolTimer = new ElapsedTime();
                while (opModeIsActive() && robot.flywheel.getVelocity() < calculatedVel - 50) {
                    telemetry.addData("Auto Status", "Spooling: %.0f", robot.flywheel.getVelocity());
                    telemetry.update();
                    idle();
                }

                robot.setFeederPower(1.0);
                sleep(3000);
            }

        } else {
            telemetry.addData("WARNING", "Tag Not Found. Using Max Velocity.");
            robot.bankShotAuto();
            sleep(3000);
        }

        robot.setFeederPower(0);
        robot.setShooterVelocity(0);
        robot.stopDrive();
        robot.resetIntegralSum();
    }
}