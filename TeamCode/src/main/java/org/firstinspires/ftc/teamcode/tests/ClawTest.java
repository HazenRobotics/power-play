package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "ClawTest",group = "tests")
@Disabled
public class ClawTest extends OpMode {

	Claw claw;

	@Override
	public void init( ) {
		claw = new Claw( hardwareMap, "left", "right", new double[]{ 0.5, 0.75 }, new double[]{ 0.5, 0.25 } );
	}

	@Override
	public void loop( ) {
		if ( gamepad1.a )
			claw.open();
		else if ( gamepad1.b )
			claw.close();
	}
}
