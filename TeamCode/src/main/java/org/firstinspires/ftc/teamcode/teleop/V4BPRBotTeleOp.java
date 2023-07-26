package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.V4BPRBot;

@TeleOp
public class V4BPRBotTeleOp extends LinearOpMode {

	V4BPRBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new V4BPRBot( this );

		waitForStart();

		robot.drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
	}
}
