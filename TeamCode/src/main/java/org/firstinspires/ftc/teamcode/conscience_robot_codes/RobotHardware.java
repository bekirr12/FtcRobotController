package org.firstinspires.ftc.teamcode.conscience_robot_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// This file is not opmode, it is just a hardware library.
public class RobotHardware {

    // hardware objects
    public DcMotorEx flywheel = null; // DCMotorEx for velocity control
    public DcMotor coreHex = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public CRServo servo = null;
    public Servo servo2 = null;

    // constant values
    public final int BANK_VELOCITY = 1350;
    public final int FAR_VELOCITY = 1550;
    public final int MAX_VELOCITY = 2200;
    private final double WHEELS_INCHES_TO_TICKS = (28.0 * 5.0) / Math.PI; // check actual values

    // variables
    public double speedMultiplier = 0.5;
    private LinearOpMode opMode = null;
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    // It maps all the hardware of the robot and makes its initial settings.
    public void init(LinearOpMode opMode, HardwareMap hMap) {
        this.opMode = opMode;

        // hardware mapping
        try {
            flywheel = hMap.get(DcMotorEx.class, "flywheel");
            coreHex = hMap.get(DcMotor.class, "coreHex");
            leftDrive = hMap.get(DcMotor.class, "leftDrive");
            rightDrive = hMap.get(DcMotor.class, "rightDrive");
            servo = hMap.get(CRServo.class, "servo");
            servo2 = hMap.get(Servo.class, "servo2");
        } catch (Exception e) {
            opMode.telemetry.addData("Error", "Hardware mapping failed");
            opMode.telemetry.update();
            return;
        }

        // direction and mode settings
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        coreHex.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo.setPower(0);
        servo2.setPosition(0);
    }

    // REUSABLE METHODS

    // It implements arcade driving using gamepad data.
    public void splitStickArcadeDrive(double x_stick, double y_stick) {
        double x = x_stick * speedMultiplier;
        double y = y_stick * speedMultiplier;

        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    // It drives to the specified distance using the encoder.
    public void  autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        if (!opMode.opModeIsActive()) return;

        autoDriveTimer.reset();

        int leftTargetPosition = leftDrive.getCurrentPosition() + (int) (leftDistanceInch * WHEELS_INCHES_TO_TICKS);
        int rightTargetPosition = rightDrive.getCurrentPosition() + (int) (rightDistanceInch * WHEELS_INCHES_TO_TICKS);

        leftDrive.setTargetPosition(leftTargetPosition);
        rightDrive.setTargetPosition(rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (leftDrive.isBusy() || rightDrive.isBusy()) &&
                (autoDriveTimer.milliseconds() < timeout_ms)) {
            opMode.idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // Sets the launcher for close shooting and activates the feeder.
    public void bankShotAuto() {
        flywheel.setVelocity(BANK_VELOCITY);
        servo.setPower(-1);
        if (flywheel.getVelocity() >= BANK_VELOCITY - 50) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    // Sets the launcher for long throw and activates the feeder.
    public void farPowerAuto() {
        flywheel.setVelocity(FAR_VELOCITY);
        servo.setPower(-1);
        if (flywheel.getVelocity() >= FAR_VELOCITY - 100) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }
}