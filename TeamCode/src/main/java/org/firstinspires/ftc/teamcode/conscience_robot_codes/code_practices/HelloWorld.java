package org.firstinspires.ftc.teamcode.conscience_robot_codes.code_practices;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class HelloWorld extends OpMode {
    @Override
    public void init() {
        telemetry.addLine("Hello Bekir, You're welcome");
    }

    @Override
    public void loop() {

    }
}
