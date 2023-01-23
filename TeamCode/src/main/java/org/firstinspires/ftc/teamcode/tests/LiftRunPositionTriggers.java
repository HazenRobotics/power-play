package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(group = "Test")
//@Disabled
public class LiftRunPositionTriggers extends OpMode {

	Lift lift;
	double lastPos = 0;

	@Override
	public void init( ) {
		lift = new Lift( hardwareMap, "lift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 103.6, 1, new PIDController(0.02, 0, 0.00012) );
		lift.setEncoder( Lift.EncoderState.WITH_ENCODER );
	}

	@Override
	public void loop( ) {

		if( Math.abs( gamepad1.right_trigger + gamepad1.left_trigger ) > 0.05 ) {
			lift.setTeleOpPowerMode();
			lift.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
			lastPos = lift.getPositionInch( );
		} else if (Math.abs( lastPos - lift.getMotorPositionInch() ) > 0.5 ) {
			lift.setHeightPower( 0.25, lastPos, true, false  );
		}
		displayTelemetry( );
	}

	public void displayTelemetry( ) {
		telemetry.addData( "lastPos", lastPos );
		telemetry.addData( "lt", gamepad1.left_trigger );
		telemetry.addData( "rt", gamepad1.right_trigger );
		telemetry.addData( "pow", lift.getPower() );
		telemetry.update( );
	}
}
