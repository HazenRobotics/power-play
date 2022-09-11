package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CRServoTester", group = "TeleOp")
//@Disabled
public class CRServoTester extends OpMode {

    CRServo servo;

    double power = 0;

    @Override
    public void init() {
        servo = hardwareMap.crservo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            power = 0.5;
        else if (gamepad1.b)
            power = -0.5;

        telemetry.addData("position: ", power);
        telemetry.update();
        servo.setPower(power);
        power = 0;
    }
}
