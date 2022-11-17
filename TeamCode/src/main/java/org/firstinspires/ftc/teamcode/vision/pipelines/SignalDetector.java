package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
			new Point( 500, 350 ),
			new Point( 570, 450 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.02;

	public SignalDetector( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input ) {

		//setting up green mat
		Imgproc.cvtColor( input, greenMat, Imgproc.COLOR_RGB2HSV );

		Scalar greenLowHSV = new Scalar(67.0, 25.0, 0.0);
		Scalar greenHighHSV = new Scalar(77.0, 255.0, 255.0);

		Core.inRange( greenMat, greenLowHSV, greenHighHSV, greenMat );

		Mat smallGreenMat = greenMat.submat( ROI );

		//setting up yellow mat
		Imgproc.cvtColor( input, yellowMat, Imgproc.COLOR_RGB2HSV );

		Scalar yellowLowHSV = new Scalar(26.0, 25.0, 0.0);
		Scalar yellowHighHSV = new Scalar(36.0, 255.0, 255.0);

		Core.inRange( yellowMat, yellowLowHSV, yellowHighHSV, yellowMat );

		Mat smallYellowMat = yellowMat.submat( ROI );

		//setting up brown mat
		Imgproc.cvtColor( input, pinkMat, Imgproc.COLOR_RGB2HSV );

		Scalar pinkLowHSV = new Scalar(159.0, 25.0, 0.0);
		Scalar pinkHighHSV = new Scalar(169.0, 255.0, 255.0);

		Core.inRange( pinkMat, pinkLowHSV, pinkHighHSV, pinkMat );

		Mat smallPinkMat = pinkMat.submat( ROI );


//		Solid Color HSVs

//		Scalar( 55, 25, 25 );
//		Scalar( 70, 255, 255 );
//
//		Scalar( 30, 30, 30 );
//		Scalar( 40, 175, 175 );
//
//		Scalar( 150, 125, 125 );
//		Scalar( 170, 255, 255 );


		double greenValue = Core.sumElems( smallGreenMat ).val[0] / ROI.area( ) / 255;
		double yellowValue = Core.sumElems( smallYellowMat ).val[0] / ROI.area( ) / 255;
		double pinkValue = Core.sumElems( smallPinkMat ).val[0] / ROI.area( ) / 255;

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
			signalPosition = SignalPosition.MIDDLE;
			telemetry.addLine( "Green Seen" );
		} else
			telemetry.addLine( "Green Not Seen" );

		if( yellowBool ) {
			signalPosition = SignalPosition.LEFT;
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

		Imgproc.rectangle( filters[whichMat], ROI, new Scalar (255,0,0) );

		return filters[whichMat];
	}

	public SignalPosition getSignalPosition( ) {
		return signalPosition;
	}

}