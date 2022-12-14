package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Scanner;

@Config
public class PoleDetector extends OpenCvPipeline {

	Telemetry telemetry;
	Mat leftMat = new Mat( );
	Mat poleMat = new Mat( );
	Mat rightMat = new Mat( );


	Scalar poleLowHSV;
	Scalar poleHighHSV;

	public static int LEFT_ROI_X_MIN = 0;
	public static int LEFT_ROI_X_MAX = 540;

	public static int POLE_ROI_X_MIN = 541;
	public static int POLE_ROI_X_MAX = 739;

	public static int RIGHT_ROI_X_MIN = 740;
	public static int RIGHT_ROI_X_MAX = 1280;

	public enum PolePosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}

	private PolePosition polePosition;

	static final Rect LEFT_ROI = new Rect(
			new Point( 0, 0 ),
			new Point( 540, 720 ) );
	static final Rect POLE_ROI = new Rect(
			new Point( 541, 0 ),
			new Point( 739, 720 ) );
	static final Rect RIGHT_ROI = new Rect(
			new Point( 740, 0 ),
			new Point( 1280, 720 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.15;

	public PoleDetector( Telemetry t ) {
		telemetry = t;
//		readInScalars( );
	}

	public Mat processFrame( Mat input ) {

		//setting up yellow mat
		Imgproc.cvtColor( input, input, Imgproc.COLOR_RGB2HSV );

		poleLowHSV = new Scalar( 16, 100, 140 );
		poleHighHSV = new Scalar( 26, 200, 255 );

		Core.inRange( input, poleLowHSV, poleHighHSV, input );

		leftMat = input.submat( LEFT_ROI );
		poleMat = input.submat( POLE_ROI );
		rightMat = input.submat( RIGHT_ROI );

		double leftValue = Core.sumElems( leftMat ).val[0] / LEFT_ROI.area( ) / 255;
		double poleValue = Core.sumElems( poleMat ).val[0] / POLE_ROI.area( ) / 255;
		double rightValue = Core.sumElems( rightMat ).val[0] / RIGHT_ROI.area( ) / 255;

		leftMat.release( );
		poleMat.release( );
		rightMat.release( );


		boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
		boolean poleBool = poleValue > PERCENT_COLOR_THRESHOLD;
		boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

		if( leftBool ) {
			polePosition = PolePosition.LEFT;
			telemetry.addLine( "Pole seen on left" );
		} else
			telemetry.addLine( "No pole on the left" );

		if( poleBool ) {
			polePosition = PolePosition.MIDDLE;
			telemetry.addLine( "Pole seen in the middle" );
		} else
			telemetry.addLine( "No Pole in the middle" );

		if( rightBool ) {
			polePosition = PolePosition.RIGHT;
			telemetry.addLine( "Pole seen on right" );
		} else
			telemetry.addLine( "No pole on the right" );

		if( !leftBool && !poleBool && !rightBool ) {
			polePosition = PolePosition.NOT_FOUND;
			telemetry.addLine( "Nothing There" );
		}

		telemetry.update( );

		Imgproc.rectangle( input, LEFT_ROI, new Scalar( 25, 255, 255 ) );
		Imgproc.rectangle( input, POLE_ROI, new Scalar( 70, 255, 255 ) );
		Imgproc.rectangle( input, RIGHT_ROI, new Scalar( 90, 255, 255 ) );

		return input;
	}

	public PolePosition getPolePosition( ) {
		return polePosition;
	}

}