package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalDetector extends OpenCvPipeline {

	Telemetry telemetry;
	Mat greenMat = new Mat( );
	Mat yellowMat = new Mat( );
	Mat pinkMat = new Mat( );

	public enum SignalPosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}

	private SignalPosition signalPosition;

	static final Rect ROI = new Rect(
			new Point( 0, 0 ),
			new Point( 1280, 720 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.02;

	public SignalDetector( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input ) {

		//setting up green mat
		Imgproc.cvtColor( input, greenMat, Imgproc.COLOR_RGB2HSV );

		Scalar greenLowHSV = new Scalar( 45, 25, 25 );
		Scalar greenHighHSV = new Scalar( 75, 255, 255 );

		Core.inRange( greenMat, greenLowHSV, greenHighHSV, greenMat );

		//setting up yellow mat
		Imgproc.cvtColor( input, yellowMat, Imgproc.COLOR_RGB2HSV );

		Scalar yellowLowHSV = new Scalar( 30, 75, 75 );
		Scalar yellowHighHSV = new Scalar( 40, 255, 255 );

		Core.inRange( yellowMat, yellowLowHSV, yellowHighHSV, yellowMat );

		//setting up brown mat
		Imgproc.cvtColor( input, pinkMat, Imgproc.COLOR_RGB2HSV );

		Scalar pinkLowHSV = new Scalar( 140, 55, 0 );
		Scalar pinkHighHSV = new Scalar( 175, 255, 255 );

		Core.inRange( pinkMat, pinkLowHSV, pinkHighHSV, pinkMat );

//		Solid Color HSVs

//		Scalar( 55, 25, 25 );
//		Scalar( 70, 255, 255 );
//
//		Scalar( 30, 30, 30 );
//		Scalar( 40, 175, 175 );
//
//		Scalar( 150, 125, 125 );
//		Scalar( 170, 255, 255 );


		double greenValue = Core.sumElems( greenMat ).val[0] / ROI.area( ) / 255;
		double yellowValue = Core.sumElems( yellowMat ).val[0] / ROI.area( ) / 255;
		double pinkValue = Core.sumElems( pinkMat ).val[0] / ROI.area( ) / 255;

//		greenMat.release( );
//		yellowMat.release( );
//		brownMat.release( );

		boolean greenBool = greenValue > PERCENT_COLOR_THRESHOLD;
		boolean yellowBool = yellowValue > PERCENT_COLOR_THRESHOLD;
		boolean pinkBool = pinkValue > PERCENT_COLOR_THRESHOLD;

		if( pinkBool ) {
			signalPosition = SignalPosition.RIGHT;
			telemetry.addLine( "Pink Seen" );
		} else
			telemetry.addLine( "Pink Not Seen" );

		if( greenBool ) {
			signalPosition = SignalPosition.LEFT;
			telemetry.addLine( "Green Seen" );
		} else
			telemetry.addLine( "Green Not Seen" );

		if( yellowBool ) {
			signalPosition = SignalPosition.MIDDLE;
			telemetry.addLine( "Yellow Seen" );
		} else
			telemetry.addLine( "Yellow Not Seen" );

		if( !greenBool && !pinkBool && !yellowBool ) {
			signalPosition = SignalPosition.NOT_FOUND;
			telemetry.addLine( "Nothing There" );
		}


//		Imgproc.cvtColor( greenMat, greenMat, Imgproc.COLOR_GRAY2RGB );

		telemetry.update( );

		Mat[] filters = { greenMat, pinkMat, yellowMat };

		int whichMat = (int) ((System.currentTimeMillis( ) / 1000) % 3);

		if( whichMat == 0 )
			telemetry.addLine( "Showing Green" );
		else if( whichMat == 1 )
			telemetry.addLine( "Showing Pink" );
		else if( whichMat == 2 )
			telemetry.addLine( "Showing Yellow" );

		return filters[whichMat];
	}

	public SignalPosition getSignalPosition( ) {
		return signalPosition;
	}

}