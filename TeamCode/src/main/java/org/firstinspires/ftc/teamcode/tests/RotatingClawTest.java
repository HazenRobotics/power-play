package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.RotatingClaw;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(group = "Test")
//@Disabled
public class RotatingClawTest extends OpMode {

	RotatingClaw claw;
	GamepadEvents controller1;
	boolean opened = true;

	@Override
	public void init( ) {
		Robot.writeToDefaultFile( "written", false, true );

		claw = new RotatingClaw( hardwareMap, "claw", "clawR", new double[]{0.35,0.65} );
		claw.rotate( 0.5 );
		controller1 = new GamepadEvents( gamepad1 );
	}

	@Override
	public void loop( ) {
		if( controller1.a.onPress( ) ) {
			telemetry.addLine( "on a press" );
			if( opened )
				claw.close( );
			else
				claw.open( );
			opened = !opened;
		}

		if( controller1.left_bumper.getValue() )
			claw.rotate( claw.getRotatePos() - 0.05 );
		else if( controller1.right_bumper.getValue() )
			claw.rotate( claw.getRotatePos() + 0.05 );

		telemetry.addData( "lb", controller1.left_bumper.getValue() );
		telemetry.addData( "rb", controller1.right_bumper.getValue() );
		telemetry.addData( "claw position", claw.getRotatePos() );

		controller1.update();
	}
}
