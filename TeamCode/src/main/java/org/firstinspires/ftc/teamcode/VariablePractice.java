package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int teamNumber = 23014;
        int motorAngle = 180;
        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String teamName = "ConScience";

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Angle", motorAngle);
        telemetry.addData("motorSpeed", motorSpeed);
        telemetry.addData("Claw Closed", clawClosed);
        telemetry.addData("Team Name", teamName);

    }
    @Override
    public void loop() {
        /*
        1. Change the String variable "name" to your team name.
        2. Create an int called "motorAngle" and store an angle between 0-180. display this your init method.
         */
    }

}
