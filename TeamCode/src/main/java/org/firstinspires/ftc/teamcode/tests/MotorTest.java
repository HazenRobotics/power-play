package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get( DcMotor.class, "motor" );
    }

    @Override
    public void loop() {

    }
}
