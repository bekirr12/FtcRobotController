package org.firstinspires.ftc.teamcode.conscience_robot_codes.code_practices;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamePadPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // runs 50x a second
        double speedForward = -gamepad1.left_stick_y / 2.0;
        double differenceXJoysticks = gamepad1.left_stick_x - gamepad1.right_stick_x;

        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", speedForward);
        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", gamepad1.right_stick_y);

        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);

        telemetry.addData("Difference between x value", differenceXJoysticks);

    }
}
