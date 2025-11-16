package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class IfPractice extends OpMode {
    @Override
    public void init() {
        /*
        == is equal to
        > is greater than
        < is less than
        >= is greater than or equal to
        <= is less than or equal to
        != is not equal to

        AND - && if (leftY < 0.5 && leftY > 0)
        OR - || if (leftY < 0.5 || leftY > 0)
        NOT - ! if (!clawClosed)
         */

    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;

        if (leftY < 0) {
            telemetry.addData("Left Stick", "is Negative");
        } else if (leftY > 0.5) {
            telemetry.addData("Left Stick", "is greater than 50%");
        } else if (leftY > 0) {
            telemetry.addData("Left Stick", "is greater than Zero");
        }  else {
            telemetry.addData("Left Stick", "is Zero");
        }
        telemetry.addData("Left Stick value", leftY);
    }
}
