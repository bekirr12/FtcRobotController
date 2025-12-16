package org.firstinspires.ftc.teamcode.conscience_robot_codes.piri_reis_kodları;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "REVStarterBotTeleOpAuto_birinci_kod (Blocks to Java)")

public class REVStarterBotTeleOpAuto_birinci_kod extends LinearOpMode {



    private DcMotor flywheel;

    private DcMotor coreHex;

    private DcMotor leftDrive;

    private CRServo servo;

    private Servo servo2;

    private DcMotor rightDrive;



    int bankVelocity;

    double speedMultiplier;

    int farVelocity;

    ElapsedTime autoDriveTimer;

    String operationSelected;

    int maxVelocity;

    String TELEOP;

    String AUTO_BLUE;

    String AUTO_RED;

    double WHEELS_INCHES_TO_TICKS;

    ElapsedTime autoLaunchTimer;



    /**

     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue

     * Comment Blocks show where to place Initialization code (runs once, after touching the

     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after

     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not

     * Stopped).

     */

    @Override

    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        coreHex = hardwareMap.get(DcMotor.class, "coreHex");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");

        servo = hardwareMap.get(CRServo.class, "servo");

        servo2 = hardwareMap.get(Servo.class, "servo2");

        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");



        initRobot();

        while (opModeInInit()) {

            operationSelected = selectOperation(operationSelected, gamepad1.psWasPressed());

            telemetry.update();

        }

        waitForStart();

        if (operationSelected.equals(AUTO_BLUE)) {

            doAutoBlue();

        } else if (operationSelected.equals(AUTO_RED)) {

            doAutoRed();

        } else {

            doTeleOp();

        }

    }



    /**

     * Describe this function...

     */

    private void initRobot() {

// Setting the direction and mode for the motors

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);

        coreHex.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        servo.setPower(0);

        servo2.setPosition(0);

        bankVelocity = 1350;

        farVelocity = 1550;

        maxVelocity = 2200;

        speedMultiplier = 0.5;

        AUTO_BLUE = "AUTO BLUE";

        AUTO_RED = "AUTO RED";

        TELEOP = "TELEOP";

        operationSelected = TELEOP;

        autoLaunchTimer = new ElapsedTime();

        autoDriveTimer = new ElapsedTime();

        WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);

    }



    /**

     * Describe this function...

     */

    private void doTeleOp() {

        if (opModeIsActive()) {

            while (opModeIsActive()) {

// Calling our functions while the OpMode is running

                updateSpeedMultiplier();

                splitStickArcadeDrive();

                setFlywheelVelocity();

                manualServoAndCoreHexControl();

                smartServoGate();

                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());

                telemetry.addData("Flywheel Power", flywheel.getPower());

                telemetry.update();

            }

        }

    }



    /**

     * Describe this function...

     */

    private void bankShotAuto() {

        ((DcMotorEx) flywheel).setVelocity(bankVelocity);

        servo.setPower(-1);

        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {

            coreHex.setPower(1);

        } else {

            coreHex.setPower(0);

        }

    }



    /**

     * Describe this function...

     */

    private void updateSpeedMultiplier() {

        if (gamepad2.dpadUpWasPressed()) {

            speedMultiplier = speedMultiplier + 0.1;

            if (speedMultiplier > 1) {

                speedMultiplier = 1;

            }

        } else if (gamepad2.dpadDownWasPressed()) {

            speedMultiplier = speedMultiplier - 0.1;

            if (speedMultiplier < 0) {

                speedMultiplier = 0;

            }

        }

    }



    /**

     * Describe this function...

     */

    private void farPowerAuto() {

        ((DcMotorEx) flywheel).setVelocity(farVelocity);

        servo.setPower(-1);

        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {

            coreHex.setPower(1);

        } else {

            coreHex.setPower(0);

        }

    }



    /**

     * Describe this function...

     */

    private String selectOperation(String state, boolean cycleNext) {

        if (cycleNext) {

            if (state.equals(TELEOP)) {

                state = AUTO_BLUE;

            } else if (state.equals(AUTO_BLUE)) {

                state = AUTO_RED;

            } else if (state.equals(AUTO_RED)) {

                state = TELEOP;

            } else {

                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");

            }

        }

        telemetry.addLine("Press Home Button to cycle options");

        telemetry.addData("CURRENT SELECTION", state);

        if (state.equals(AUTO_BLUE) || state.equals(AUTO_RED)) {

            telemetry.addLine("Please remember to enable the AUTO timer!");

        }

        telemetry.addLine("Press START to start your program");

        return state;

    }



    /**

     * Describe this function...

     */

    private void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {

        autoDriveTimer.reset();

        leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));

        rightDrive.setTargetPosition((int) (rightDrive.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));

        rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {

            idle();

        }

        leftDrive.setPower(0);

        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    /**

     * Describe this function...

     */

    private void setFlywheelVelocity() {

        if (gamepad1.options) {

            flywheel.setPower(-0.5);

        } else if (gamepad1.left_bumper) {

            farPowerAuto();

        } else if (gamepad1.right_bumper) {

            bankShotAuto();

        } else if (gamepad1.circle) {

            ((DcMotorEx) flywheel).setVelocity(bankVelocity);

        } else if (gamepad1.square) {

            ((DcMotorEx) flywheel).setVelocity(maxVelocity);

        } else {

            ((DcMotorEx) flywheel).setVelocity(0);

            coreHex.setPower(0);

// The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!

            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {

                servo.setPower(0);

            }

        }

    }



    /**

     * Describe this function...

     */

    private void splitStickArcadeDrive() {

        float x;

        float y;



        x = gamepad2.left_stick_x;

        y = gamepad2.left_stick_y;

        x = (float) (x * speedMultiplier);

        y = (float) (y * speedMultiplier);

        leftDrive.setPower(y - x);

        rightDrive.setPower(y + x);

    }



    /**

     * Describe this function...

     */

    private void doAutoBlue() {

        if (opModeIsActive()) {

            telemetry.addData("RUNNING OPMODE", operationSelected);

            telemetry.update();

            autoDrive(0.5, 37, 37, 2000);

// Fire balls

            autoLaunchTimer.reset();

            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 7000) {

                bankShotAuto();

                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());

                telemetry.update();

            }

            ((DcMotorEx) flywheel).setVelocity(0);

            coreHex.setPower(0);

            servo.setPower(0);

            autoDrive(0.5, -11, 7, 5000);

            autoDrive(0.5, 50, 50, 5000);

        }

    }



    /**

     * Describe this function...

     */

    private void doAutoRed() {

        if (opModeIsActive()) {

            telemetry.addData("RUNNING OPMODE", operationSelected);

            telemetry.update();

            autoDrive(0.5, 37, 37, 2000);

// Fire balls

            autoLaunchTimer.reset();

            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 7000) {

                bankShotAuto();

                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());

                telemetry.update();

            }

            ((DcMotorEx) flywheel).setVelocity(0);

            coreHex.setPower(0);

            servo.setPower(0);

            autoDrive(0.5, 12, -7, 5000);

            autoDrive(0.5, 50, 50, 5000);

        }

    }



    /**

     * Describe this function...

     */

    private void manualServoAndCoreHexControl() {

// Manual control for the Core Hex feeder

        if (gamepad1.cross) {

            coreHex.setPower(0.5);

        } else if (gamepad1.triangle) {

            coreHex.setPower(-0.5);

        }

// Manual control for the hopper's servo

        if (gamepad1.dpad_left) {

            servo.setPower(1);

        } else if (gamepad1.dpad_right) {

            servo.setPower(-1);

        }

    }



    /**

     * Describe this function...

     */

    private void smartServoGate() {

        if (gamepad1.dpad_up) {

            servo2.setPosition(0.5);

        } else if (gamepad1.dpad_down) {

            servo2.setPosition(0);

        }

    }

}
