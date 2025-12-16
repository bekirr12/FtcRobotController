package org.firstinspires.ftc.teamcode.conscience_robot_codes.turkey_finals_blue_uzak;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 2025 FTC DECODE sezonunda Türkiye Finalleri için kullanilacak kod.

 FTC Yazilim Ekibi
 Bekir Sami Karatas
 27.11.2025
 */
@TeleOp(name = "TR_Finalleri_Kodu_blue_uzak", group = "Competition")
public class Auto_and_TeleOp extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private AutonomousController autoController;

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
    private final String CALIBRATION = "CALIBRATION";

    private String operationSelected = TELEOP;
    private double activeShotVelocity = 0;

    // MAIN EXECUTION FLOW (runOpMode)
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, hardwareMap);
        autoController = new AutonomousController(robot, this);
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
            } else if (operationSelected.equals(CALIBRATION)) {
                doCalibrationMode();
            } else {
                doTeleOpStateful();
            }

            telemetry.addData("Flywheel Vel", robot.flywheel.getVelocity());
            telemetry.addData("left power", robot.leftDrive.getPower());
            telemetry.addData("right power", robot.rightDrive.getPower());
            telemetry.addData("hex motor power", robot.coreHex.getPower());
            telemetry.update();
        }
    }

    /**
     * YENİ: Kalibrasyon modu
     * Manuel olarak turret açısı ayarlamak için
     */
    private void doCalibrationMode() {
        telemetry.addData("Mode", "CALIBRATION");
        // telemetry.addLine("D-pad Up/Down: Adjust turret");
        telemetry.addLine("Circle: Exit");
        telemetry.update();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // Dpad Yukarı basıldıysa hızı 50 artır (Tek seferlik işlem)
            if (currentDpadUp && !lastDpadUp) {
                robot.adjustBankVelocity(10);
            }
            // Dpad Aşağı basıldıysa hızı 50 azalt (Tek seferlik işlem)
            if (currentDpadDown && !lastDpadDown) {
                robot.adjustBankVelocity(-10);
            }

            // Tuş durumlarını kaydet
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // --- ATEŞLEME KISMI (BANK SHOT AUTO) ---
            if (gamepad1.right_trigger > 0.5) {
                // Tetik basılıyken senin istediğin fonksiyonu çağırıyoruz
                // Bu fonksiyon artık güncel 'robot.bankVelocity' değerini kullanacak
                robot.bankShotAuto();

                telemetry.addData("Status", "FIRING (BankAuto)");
            } else {
                // Tetik basılı değilse her şeyi durdur
                robot.flywheel.setVelocity(0);
                robot.coreHex.setPower(0);
                robot.hopperServo.setPower(0);

                telemetry.addData("Status", "Standby");
            }
            telemetry.addData("CURRENT BANK VELOCITY", robot.bankVelocity); // Ayarlanan hızı gör
            telemetry.addData("Actual Flywheel Vel", robot.flywheel.getVelocity());
            telemetry.addData("Actual Hex Power", robot.coreHex.getPower());

            double x_s = gamepad2.left_stick_x;
            double y_s  = -gamepad2.left_stick_y;
            robot.splitStickArcadeDrive(x_s, y_s); // Uses speedMultiplier internally



            AprilTagDetection tag = robot.getLatestTargetDetection();
            if (tag != null) {
                double range = tag.ftcPose.range;
                double position = tag.ftcPose.bearing;
                // double currentAngle = robot.getCalculatedTurretAngle();

                telemetry.addData("Range", "%.1f inch", range);
                telemetry.addData("Bearing Error", "%.1f°", position);
                // telemetry.addData("Turret Current Angle", "%.3f", robot.turretServo.getPosition());




            } else {
                telemetry.addData("Status", "No AprilTag detected");
            }

            // Exit
            if (gamepad1.circle) {
                break;
            }

            telemetry.update();
            idle();
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
            telemetry.addData("Range", "%.1f in", tag.ftcPose.range);
            telemetry.addData("Bearing Error", "%.1f°", tag.ftcPose.bearing);
            telemetry.addData("Target Vel", "%.0f", robot.getCalculatedShotVelocity());
            // telemetry.addData("Target Angle", "%.3f", robot.getCalculatedTurretAngle());
        } else {
            gamepad1.rumble(1.0, 1.0, 200);
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

        if (robot.flywheel.getVelocity() >= activeShotVelocity - 5) {
            robot.setFeederPower(1.0);
            sleep(250);
            robot.setFeederPower(0);
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
        // robot.resetTurret();
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
            case "AUTO RED": state = CALIBRATION; break;
            case "CALIBRATION": state = "TELEOP"; break; // "CALIBRATION" -> "TELEOP"
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
        executeAutonomousSequence();
    }
    private void doAutoRed() throws InterruptedException {
        executeAutonomousSequence();
    }

    /**
     * Event-driven autonomous loop
     * Continues until all shots fired and driven to line
     */
    private void executeAutonomousSequence() {
        while (opModeIsActive() && autoController.executeAutoSequence()) {
            // Telemetry
            telemetry.addData("State", autoController.getCurrentState());
            telemetry.addData("Shots Fired", autoController.getShotsCompleted());
            telemetry.addData("Flywheel Vel", robot.flywheel.getVelocity());
            telemetry.addData("Left Power", robot.leftDrive.getPower());
            telemetry.addData("Right Power", robot.rightDrive.getPower());
            telemetry.update();

            idle();
        }

        // Cleanup
        robot.setFeederPower(0);
        robot.setShooterVelocity(0);
        robot.stopDrive();
        robot.resetIntegralSum();

        telemetry.addData("Status", "AUTONOMOUS COMPLETE");
        telemetry.update();
    }

}