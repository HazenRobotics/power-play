package org.firstinspires.ftc.teamcode.vision;

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
	Mat mat = new Mat( );
	Mat greenMat = new Mat( );
//	Mat yellowMat = new Mat( );
//	Mat brownMat = new Mat( );

	public enum SignalPosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}

	private SignalPosition signalPosition;

	static final Rect GREEN_ROI = new Rect(
			new Point( 426, 0 ),
			new Point( 852, 720 ) );
	static final Rect YELLOW_ROI = new Rect(
			new Point( 426, 0 ),
			new Point( 852, 720 ) );
	static final Rect BROWN_ROI = new Rect(
			new Point( 426, 0 ),
			new Point( 852, 720 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.02;

	public SignalDetector( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input) {

		Imgproc.cvtColor( input, mat, Imgproc.COLOR_RGB2HSV );

		Scalar greenLowHSV = new Scalar( 40, 20, 255 );
		Scalar greenHighHSV = new Scalar( 65, 255, 255 );

//		Scalar yellowLowHSV = new Scalar( 40, 50, 70 );
//		Scalar yellowHighHSV = new Scalar( 65, 255, 255 );
//
//		Scalar brownLowHSV = new Scalar( 40, 50, 70 );
//		Scalar brownHighHSV = new Scalar( 65, 255, 255 );

		Core.inRange( mat, greenLowHSV, greenHighHSV, greenMat );
//		Core.inRange( mat, yellowLowHSV, yellowHighHSV, yellowMat );
//		Core.inRange( mat, brownLowHSV, brownHighHSV, brownMat );

		greenMat = mat.submat( GREEN_ROI );
//		yellowMat = mat.submat( YELLOW_ROI );
//		brownMat = mat.submat( BROWN_ROI );

		double greenValue = Core.sumElems( greenMat ).val[0] / GREEN_ROI.area( ) / 255;
//		double yellowValue = Core.sumElems( yellowMat ).val[0] / YELLOW_ROI.area( ) / 255;
//		double brownValue = Core.sumElems( brownMat ).val[0] / BROWN_ROI.area( ) / 255;

		greenMat.release( );
//		yellowMat.release( );
//		brownMat.release( );

		boolean greenBool = greenValue > PERCENT_COLOR_THRESHOLD;
//		boolean yellowBool = yellowValue > PERCENT_COLOR_THRESHOLD;
//		boolean brownBool = brownValue > PERCENT_COLOR_THRESHOLD;

//		if( brownBool ) {
//			signalPosition = SignalPosition.RIGHT;
//			telemetry.addData( "Location", " right" );
//		} else if( greenBool ) {
//			signalPosition = SignalPosition.LEFT;
//			telemetry.addData( "Location", " left" );
//		} else if( yellowBool ) {
//			signalPosition = SignalPosition.MIDDLE;
//			telemetry.addData( "Location", " middle" );
//		} else {
//			signalPosition = SignalPosition.NOT_FOUND;
//			telemetry.addData( "Location", " not found" );
//		}

		if (greenBool) {
			signalPosition = SignalPosition.LEFT;
			telemetry.addData( "Location", " left" );
		} else {
			signalPosition = SignalPosition.NOT_FOUND;
			telemetry.addData( "Location", " not found" );
		}

//		Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );
	
		telemetry.update();
		return greenMat;
	}

	public SignalPosition getSignalPosition( ) {
		return signalPosition;
	}
}