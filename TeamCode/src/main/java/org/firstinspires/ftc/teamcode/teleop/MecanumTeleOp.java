package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
//@Disabled
public class MecanumTeleOp extends OpMode {

    MecanumDrive drive;

    @Override
    public void init( ) {
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop( ) {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
}