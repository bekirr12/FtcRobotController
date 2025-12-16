package org.firstinspires.ftc.teamcode.conscience_robot_codes.code_practices;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp
public class IMUPractice extends OpMode {
    TestIMU bench = new TestIMU();


    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Heading", bench.getHeading(AngleUnit.DEGREES));
    }
}
