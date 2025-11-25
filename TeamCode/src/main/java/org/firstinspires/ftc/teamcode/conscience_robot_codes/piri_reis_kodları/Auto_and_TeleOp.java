package org.firstinspires.ftc.teamcode.conscience_robot_codes.piri_reis_kodları;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main competition OpMode, manages Autonomous selection during INIT and executes
 * the chosen mode (Auto or TeleOp) during RUN. It relies on RobotHardware.java
 * for all motor control and hardware logic.
 */
@Disabled
@Autonomous(name = "Auto_and_TeleOp", group = "Competition")
public class Auto_and_TeleOp extends LinearOpMode {

    // Create an instance of the RobotHardware class to access all hardware and functions
    private RobotHardware robot = new RobotHardware();

    // Constants for OpMode Selection
    private final String AUTO_BLUE = "AUTO BLUE";
    private final String AUTO_RED = "AUTO RED";
    private final String TELEOP = "TELEOP";

    // Variables and Timers
    private String operationSelected = TELEOP;
    private ElapsedTime autoLaunchTimer = new ElapsedTime();


    // =========================================================================
    // MAIN OPMODE METHOD
    // =========================================================================

    @Override
    public void runOpMode() throws InterruptedException {

        // A. Initialize Robot Hardware
        // Calls the init method in the RobotHardware class to map and configure all devices.
        robot.init(this, hardwareMap);
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        // B. Autonomous/TeleOp Selection Loop (INIT phase)
        while (!isStarted() && !isStopRequested()) {

            // Selection mechanism: Cycle through modes when the 'options' button is pressed
            if (gamepad1.psWasPressed()) {
                operationSelected = selectOperation(operationSelected, true);
                sleep(250); // Debounce delay to prevent multiple selections
            }

            updateSelectionTelemetry();
            idle();
        }

        // C. Wait for START and Execute
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("RUNNING MODE", operationSelected);
            telemetry.update();

            // Execute the selected mode
            if (operationSelected.equals(AUTO_BLUE)) {
                doAutoBlue();
            } else if (operationSelected.equals(AUTO_RED)) {
                doAutoRed();
            } else {
                doTeleOp();
            }
        }
    }

    // =========================================================================
    // MODE EXECUTION METHODS
    // =========================================================================

    /** Executes the continuous TeleOp loop, handling all gamepad input logic. */
    private void doTeleOp() {
        while (opModeIsActive()) {

            // A. Drive control logic (delegated to RobotHardware)
            robot.splitStickArcadeDrive(gamepad2.left_stick_x, gamepad2.left_stick_y);

            // B. Speed Multiplier Update (modifies the public variable in RobotHardware)
            updateSpeedMultiplier();

            // C. Mechanism Controls (Gamepad logic here, hardware calls to robot object)
            setFlywheelVelocity();
            manualServoAndCoreHexControl();
            smartServoGate();

            // D. Telemetry Update
            telemetry.addData("Speed Multiplier", robot.speedMultiplier);
            telemetry.addData("Flywheel Velocity", robot.flywheel.getVelocity());
            telemetry.update();
        }
    }

    /** Autonomous sequence for the BLUE alliance. */
    private void doAutoBlue() {
        if (!opModeIsActive()) return;

        // 1. First Drive (forward 37 inches)
        robot.autoDrive(0.5, 37, 37, 2000);

        // 2. Launcher Sequence Loop (7 seconds)
        autoLaunchTimer.reset();
        while (opModeIsActive() && autoLaunchTimer.milliseconds() < 7000) {
            robot.bankShotAuto(); // Calls the hardware logic
            telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
            telemetry.update();
            idle();
        }

        // Stop the launcher mechanisms
        robot.flywheel.setVelocity(0);
        robot.coreHex.setPower(0);
        robot.servo.setPower(0);

        // 3. Second and Third Drive (Turn and Final drive)
        robot.autoDrive(0.5, -11, 7, 5000);
        robot.autoDrive(0.5, 50, 50, 5000);
    }

    /** Autonomous sequence for the RED alliance. */
    private void doAutoRed() {
        if (!opModeIsActive()) return;

        // 1. First Drive (forward 37 inches)
        robot.autoDrive(0.5, 37, 37, 2000);

        // 2. Launcher Sequence Loop (7 seconds)
        autoLaunchTimer.reset();
        while (opModeIsActive() && autoLaunchTimer.milliseconds() < 7000) {
            robot.bankShotAuto();
            telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
            telemetry.update();
            idle();
        }

        // Stop the launcher mechanisms
        robot.flywheel.setVelocity(0);
        robot.coreHex.setPower(0);
        robot.servo.setPower(0);

        // 3. Second and Third Drive (Turn and Final drive)
        robot.autoDrive(0.5, 12, -7, 5000);
        robot.autoDrive(0.5, 50, 50, 5000);
    }

    // =========================================================================
    // GAMEPAD LOGIC METHODS (Residing in the OpMode file)
    // =========================================================================

    /** Updates the robot's speed multiplier based on D-Pad input. */
    private void updateSpeedMultiplier() {
        if (gamepad2.dpadUpWasPressed()) {
            robot.speedMultiplier += 0.1;
            if (robot.speedMultiplier > 1) {
                robot.speedMultiplier = 1;
            }
        } else if (gamepad2.dpadDownWasPressed()) {
            robot.speedMultiplier -= 0.1;
            if (robot.speedMultiplier < 0) {
                robot.speedMultiplier = 0.1;
            }
        }
    }

    /** Controls the flywheel velocity based on gamepad button presses. */
    private void setFlywheelVelocity() {
        // Options button uses raw Power (as per original blocks)
        if (gamepad1.options) {
            robot.flywheel.setPower(-0.5);
        }
        // Bumpers call full autonomous shot sequences
        else if (gamepad1.left_bumper) {
            robot.farPowerAuto();
        } else if (gamepad1.right_bumper) {
            robot.bankShotAuto();
        }
        // Circle/Square set specific velocities
        else if (gamepad1.circle) {
            robot.flywheel.setVelocity(robot.BANK_VELOCITY);
        } else if (gamepad1.square) {
            robot.flywheel.setVelocity(robot.MAX_VELOCITY);
        }
        // Default: stop mechanisms
        else {
            robot.flywheel.setVelocity(0);
            robot.coreHex.setPower(0);
            // Check to stop the Continuous Servo only if D-pad isn't being pressed
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                robot.servo.setPower(0);
            }
        }
    }

    /** Manual control for the Core Hex feeder and CRServo (hopper servo). */
    private void manualServoAndCoreHexControl() {
        // Core Hex control
        if (gamepad1.cross) {
            robot.coreHex.setPower(0.5);
        } else if (gamepad1.triangle) {
            robot.coreHex.setPower(-0.5);
        }
        // CRServo control (power-based)
        if (gamepad1.dpad_left) {
            robot.servo.setPower(1);
        } else if (gamepad1.dpad_right) {
            robot.servo.setPower(-1);
        }
    }

    /** Controls the position-based Servo Gate (servo2). */
    private void smartServoGate() {
        if (gamepad1.dpad_up) {
            robot.servo2.setPosition(0.5);
        } else if (gamepad1.dpad_down) {
            robot.servo2.setPosition(0);
        }
    }

    // ------------------- TELEMETRY AND SELECTION METHODS -------------------

    /** Cycles the operation mode (TeleOp, Auto Blue, Auto Red). */
    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP)) {
                state = AUTO_BLUE;
            } else if (state.equals(AUTO_BLUE)) {
                state = AUTO_RED;
            } else if (state.equals(AUTO_RED)) {
                state = TELEOP;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached");
            }
        }
        return state;
    }

    /** Updates the telemetry display during the INIT phase to show the selected mode. */
    private void updateSelectionTelemetry() {
        telemetry.addLine("Press Options Button to Cycle Options");
        telemetry.addData("ACTIVE SELECTION", operationSelected);
        if (operationSelected.equals(AUTO_BLUE) || operationSelected.equals(AUTO_RED)) {
            telemetry.addLine("Please remember to enable the AUTO timer!");
        }
        telemetry.addLine("Press START to Begin Program");
        telemetry.update();
    }
}
