package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "TeleOp")
//@Disabled
public class ServoTester extends OpMode {

    Servo servo;

    double position = 0;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            position += 0.005;
        else if (gamepad1.b)
            position -= 0.005;
        else if (gamepad1.x)
            position = 0;
        else if (gamepad1.y)
            position = 1;
        else if (gamepad1.dpad_up)
            position = 0.5;

        telemetry.addData("position: ", position);
        telemetry.update();
        servo.setPosition(position);
    }
}
