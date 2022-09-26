package org.firstinspires.ftc.teamcode.preSeason;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp( name = "Learn Tele", group = "Test" )
public class LearnTele extends OpMode {

	DcMotor frontLeft, backLeft, frontRight, backRight;

	@Override
	public void init( ) {
		frontLeft = hardwareMap.dcMotor.get( "frontLeft" );
		backLeft = hardwareMap.dcMotor.get( "backLeft"  );
		frontRight = hardwareMap.dcMotor.get( "frontRight" );
		backRight = hardwareMap.dcMotor.get( "backRight" );

		frontLeft.setDirection( DcMotorSimple.Direction.REVERSE );
		backLeft.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	@Override
	public void loop( ) {
		drive( -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x );
	}

	public void drive( double drive, double rotate, double strafe ) {

		frontLeft.setPower( drive + rotate + strafe );
		backLeft.setPower( drive + rotate - strafe );
		frontRight.setPower( drive - rotate - strafe );
		backRight.setPower( drive - rotate + strafe );        //then you're gonna write this horrible piece of code in loop, right? go to loop, right?
	}
}
