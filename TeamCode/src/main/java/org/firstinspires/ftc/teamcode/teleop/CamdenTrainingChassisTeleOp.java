package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class CamdenTrainingChassisTeleOp extends LinearOpMode {

	MecanumDrive drive;
	CRServo eye1, eye2;
	Servo arm;

	GamepadEvents controller1;
	ElapsedTime timer;

	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new MecanumDrive( hardwareMap, "fl", "bl", "fr", "br" );
		eye1 = hardwareMap.get(CRServo.class, "eye1");
		eye2 = hardwareMap.get(CRServo.class, "eye2");
		arm = hardwareMap.get(Servo.class, "arm");
		controller1 = new GamepadEvents( gamepad1 );

		waitForStart();
		timer = new ElapsedTime(  );

		while (!isStopRequested()) {
			drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

//			if( controller1.a.onPress() )
//				arm.setPosition( arm.getPosition() == 1 ? 0 : 1 );

			arm.setPosition( (timer.milliseconds() % 500) > 250 ? 1 : 0 );

			if(gamepad1.x) {
				eye1.setPower( 1 );
				eye2.setPower( -1 );
			}
			controller1.update();
		}
	}

}
