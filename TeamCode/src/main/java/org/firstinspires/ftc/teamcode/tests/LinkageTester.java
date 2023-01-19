package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Linkage;

@Config
@TeleOp(group = "test")
//@Disabled
public class LinkageTester extends OpMode {

	Linkage linkage;
	public static double linkagePosition = 0;

	@Override
	public void init( ) {
		linkage = new Linkage( hardwareMap );
	}

	@Override
	public void loop( ) {
		linkagePosition += ((gamepad1.right_bumper || gamepad2.right_bumper ? 0.05 : 0) - (gamepad1.left_bumper || gamepad2.left_bumper ? 0.05 : 0));
		linkagePosition = Math.min( linkage.extensionLength, Math.max( linkagePosition, linkage.retractionLength ) );
		linkage.moveToExtensionDistance( linkagePosition );

		displayTelemetry( );
	}

	public void displayTelemetry( ) {
		telemetry.addData( "distance to angle", linkage.convertDistanceToAngle( linkagePosition ) );
		telemetry.addData( "angle to to servo pos", linkage.convertAngleToServoPos( linkage.convertDistanceToAngle( linkagePosition ) ) );
		telemetry.addLine("");
		telemetry.addData( "linkage servo pos", linkage.getServoPos( ) );
		telemetry.addData( "linkage angle", linkage.getAngle( ) );
		telemetry.addData( "linkage distance (based off of angle)", linkage.getExtensionDistance( ) );
		telemetry.addData( "linkage angle (based off of distance)", linkage.convertDistanceToAngle( linkage.convertAngleToDistance( linkage.getAngle( ) ) ) );
		telemetry.update( );
	}
}
