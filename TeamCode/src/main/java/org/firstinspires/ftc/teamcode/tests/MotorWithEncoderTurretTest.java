package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(group = "Test")
public class MotorWithEncoderTurretTest extends OpMode {

	Turret turret;
	double desiredPos;
	double desiredPow;

	@Override
	public void init( ) {
		turret = new Turret( hardwareMap, "turr", true, AngleUnit.DEGREES, 384.5, 170.0 / 30.0, 0, 360 );
	}

	@Override
	public void loop( ) {
		desiredPos += gamepad1.right_trigger - gamepad1.left_trigger;
		desiredPow += gamepad1.right_stick_y * 0.01 ;

		if ( gamepad1.b )
			turret.resetTurret();

		if (gamepad1.dpad_up && gamepad1.dpad_right)
			turret.setRotationPower( desiredPow, 45 );
		else if (gamepad1.dpad_up && gamepad1.dpad_left)
			turret.setRotationPower( desiredPow, -45 );
		else if (gamepad1.dpad_down && gamepad1.dpad_right)
			turret.setRotationPower( desiredPow, -225 );
		else if (gamepad1.dpad_down && gamepad1.dpad_left)
			turret.setRotationPower( desiredPow, -135 );
		else if( gamepad1.dpad_up )
			turret.setRotationPower( desiredPow, 0 );
		else if( gamepad1.dpad_right )
			turret.setRotationPower( desiredPow, 90 );
		else if( gamepad1.dpad_left )
			turret.setRotationPower( desiredPow, -90 );
		else if( gamepad1.dpad_down )
			turret.setRotationPower( desiredPow, -180 );


		telemetry.addData( "desired pos (heading)", desiredPos );
		telemetry.addData( "desired pos (ticks)", turret.convertHeadingToTicks( desiredPos ) );
		telemetry.addData( "actualRunToPos (ticks)", turret.motor.getTargetPosition() );
		telemetry.addData( "actualRunToPos (heading)", turret.convertTicksToHeading( turret.motor.getTargetPosition() ) );
		telemetry.addData( "desiredPow", desiredPow );
		telemetry.addData( "turret pos ticks", turret.getPosition() );
		telemetry.addData( "actualHeading", turret.getTurretHeading() );
		telemetry.update();
	}
}
