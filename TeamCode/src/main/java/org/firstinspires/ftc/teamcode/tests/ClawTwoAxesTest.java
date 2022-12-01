package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw;
import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw.ClawState;
import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw.HorizontalClawState;
import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw.VerticalClawState;

@TeleOp(name = "TwoAxesClawTest", group = "tests")
@Disabled
public class ClawTwoAxesTest extends OpMode {

	TwoAxesClaw claw;

	@Override
	public void init( ) {
		claw = new TwoAxesClaw( hardwareMap, "claw", "clawH", "clawV", new double[]{ 0.61, 0.35 }, new double[]{ 1, 0.5, 0 }, new double[]{ 0.73, 0.53, 0.3 } );
		claw.init( );

		telemetry.addLine( "init finished" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {
		if( gamepad1.a )
			claw.setState( ClawState.OPEN );
		else if( gamepad1.b )
			claw.setState( ClawState.CLOSED );
		else if( gamepad1.right_bumper )
			claw.setState( HorizontalClawState.RIGHT );
		else if( gamepad1.left_bumper )
			claw.setState( HorizontalClawState.LEFT );
		else if( gamepad1.dpad_right )
			claw.setState( HorizontalClawState.CENTER );
		else if( gamepad1.dpad_up )
			claw.setState( VerticalClawState.STOWED );
		else if( gamepad1.dpad_left )
			claw.setState( VerticalClawState.DEPLOYED );
		else if( gamepad1.dpad_down )
			claw.setState( VerticalClawState.PICKUP );

		telemetry.addLine( "claw pos: " + claw.getPos( ) );
		telemetry.addLine( "hor pos: " + claw.getHorizontalPos( ) );
		telemetry.addLine( "ver pos: " + claw.getVerticalPos( ) );
		telemetry.update( );
	}
}
