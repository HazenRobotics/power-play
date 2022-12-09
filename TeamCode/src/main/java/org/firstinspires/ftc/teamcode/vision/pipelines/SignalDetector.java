package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Scanner;

public class SignalDetector extends OpenCvPipeline {

	Telemetry telemetry;
	Mat greenMat = new Mat( );
	Mat yellowMat = new Mat( );
	Mat pinkMat = new Mat( );

	Scalar greenLowHSV;
	Scalar greenHighHSV;

	Scalar yellowLowHSV;
	Scalar yellowHighHSV;

	Scalar pinkLowHSV;
	Scalar pinkHighHSV;

	Scalar[] scalars = { yellowLowHSV, yellowHighHSV, greenLowHSV, greenHighHSV, pinkLowHSV, pinkHighHSV };

	public enum SignalPosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}

	private SignalPosition signalPosition;

	static final Rect ROI = new Rect(
			new Point( 450, 300 ),
			new Point( 720, 600 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.15;

	public SignalDetector( Telemetry t ) {
		telemetry = t;
//		readInScalars( );
	}

	public Mat processFrame( Mat input ) {

		//setting up yellow mat
		Imgproc.cvtColor( input, yellowMat, Imgproc.COLOR_RGB2HSV );

		Scalar yellowLowHSV = new Scalar( 16.0, 25.0, 0.0 );
		Scalar yellowHighHSV = new Scalar( 35.0, 255.0, 255.0 );

		Core.inRange( yellowMat, yellowLowHSV, yellowHighHSV, yellowMat );

		Mat smallYellowMat = yellowMat.submat( ROI );

		//setting up green mat
		Imgproc.cvtColor( input, greenMat, Imgproc.COLOR_RGB2HSV );

		Scalar greenLowHSV = new Scalar( 50.0, 25.0, 0.0 );
		Scalar greenHighHSV = new Scalar( 104.0, 255.0, 255.0 );

		Core.inRange( greenMat, greenLowHSV, greenHighHSV, greenMat );

		Mat smallGreenMat = greenMat.submat( ROI );

		//setting up pink mat
		Imgproc.cvtColor( input, pinkMat, Imgproc.COLOR_RGB2HSV );

		Scalar pinkLowHSV = new Scalar( 120.0, 25.0, 0.0 );
		Scalar pinkHighHSV = new Scalar( 155.0, 255.0, 255.0 );


		Core.inRange( pinkMat, pinkLowHSV, pinkHighHSV, pinkMat );

		Mat smallPinkMat = pinkMat.submat( ROI );

//		Non-Color Calibration HSVs

//		Scalar(67.0, 25.0, 0.0);
//		Scalar(77.0, 255.0, 255.0);
//
//		Scalar(26.0, 25.0, 0.0);
//		Scalar(36.0, 255.0, 255.0);
//
//		Scalar(159.0, 25.0, 0.0);
//		Scalar(169.0, 255.0, 255.0);


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

		smallGreenMat.release( );
		smallYellowMat.release( );
		smallPinkMat.release( );


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

		if( whichMat == 0 ) {
			telemetry.addLine( "Showing Green" );
			filters[1].release( );
			filters[2].release( );
		} else if( whichMat == 1 ) {
			telemetry.addLine( "Showing Pink" );
			filters[0].release( );
			filters[2].release( );
		} else if( whichMat == 2 ) {
			telemetry.addLine( "Showing Yellow" );
			filters[0].release( );
			filters[1].release( );
		}


		Imgproc.rectangle( filters[whichMat], ROI, new Scalar( 70, 255, 255 ) );

		return filters[whichMat];
	}

	public SignalPosition getSignalPosition( ) {
		return signalPosition;
	}

	public void readInScalars( ) {
		Scanner sc = new Scanner( "/sdcard/FIRST/logs/ColorScalars.txt" );

		for( int i = 0; i < 6; i++ ) {
			String line = sc.nextLine( );
			String[] values = line.substring( 1, line.length( ) - 1 ).split( "," );

//			scalars[i] = new Scalar( Double.parseDouble( values[0] ), Double.parseDouble( values[1] ), Double.parseDouble( values[2] ) );
			telemetry.addLine( line );
		}
	}

}