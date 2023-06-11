package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class LessonTester extends LinearOpMode {
	Claw placeholder = new Claw( new double[]{ 0, 1 },hardwareMap.get( Servo.class,"test" ) );
	Lesson test = placeholder; //replace with class you are testing
	Lesson working = placeholder; //replace with working answer key
	GamepadEvents contorller;
	@Override
	public void runOpMode( ) throws InterruptedException {
		contorller = new GamepadEvents( gamepad1 );
		waitForStart();
		test.test();
		while(!contorller.start.onPress()) contorller.update();
		working.test();
	}
}
