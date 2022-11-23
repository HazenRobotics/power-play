package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;
import org.firstinspires.ftc.teamcode.vision.ColorCalibratorUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.ColorCalibrator;
import org.opencv.core.Scalar;

@Autonomous(name = "Color Calibrator", group = "Test")
//@Disabled
public class ColorCalibratorTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {

		ColorCalibratorUtil detector = new ColorCalibratorUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine( "OpenCv init finished" );
		telemetry.update( );

		waitForStart( );

		while (!detector.isFinished() ) {}

		Scalar[] scalars = detector.getScalars( );

		Logger.writeAFile( "ColorScalars.txt", scalars[0] + "\n" + scalars[1] + "\n" + scalars[2] + "\n" + scalars[3] + "\n" + scalars[4] + "\n" + scalars[5] + "\n", false, false  );

		telemetry.addLine( "" + scalars[0] );
		telemetry.addLine( "" + scalars[1] );
		telemetry.addLine( "" + scalars[2] );
		telemetry.addLine( "" + scalars[3] );
		telemetry.addLine( "" + scalars[4] );
		telemetry.addLine( "" + scalars[5] );
		telemetry.update( );

		while( !isStopRequested() );
	}
}
