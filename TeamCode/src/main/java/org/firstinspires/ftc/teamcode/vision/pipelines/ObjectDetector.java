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

@Config
public class ObjectDetector extends OpenCvPipeline {

	Telemetry telemetry;
	Mat[] mats;
	boolean anyfound = false;
	boolean[] bools = new boolean[mats.length];
	double[] values = new double[rois.length];


	Scalar objectLowHSV;
	Scalar objectHighHSV;
	Scalar targetLowHSV;
	Scalar targetHighHSV;
	public static int LEFT_ROI_X_MIN = 0;
	public static int LEFT_ROI_X_MAX = 540;

	public static int POLE_ROI_X_MIN = 541;
	public static int POLE_ROI_X_MAX = 739;

	public static int RIGHT_ROI_X_MIN = 740;
	public static int RIGHT_ROI_X_MAX = 1280;
	static Rect[] rois;
	static double PERCENT_COLOR_THRESHOLD = 0.15;


	/**
	 * @param percentThreashold how much of a color needs to be in the area for it to be found
	 * @param numberOfAreas     number of different zones that the x axis is split by
	 * @param highHSV           High color (in HSV) possible for object
	 * @param lowHSV            lowest color (in HSV) possible for object
	 */
	public ObjectDetector( Telemetry t, double percentThreashold, Scalar lowHSV, Scalar highHSV, int numberOfAreas ) {
		telemetry = t;
		PERCENT_COLOR_THRESHOLD = percentThreashold;
		targetLowHSV = lowHSV;
		targetHighHSV = highHSV;
		rois = new Rect[numberOfAreas];
		int size = 1280 / numberOfAreas;
		for( int i = 0; i < rois.length; i++ ) {
			int x1 = size * i + 1;
			int x2 = size + (i * 1);
			if( i == 0 ) {
				rois[i] = new Rect( new Point( 0, 0 ), new Point( x2, 720 );
			} else {
				rois[i] = new Rect( new Point( x1, 0 ), new Point( x2, 720 ) );
			}
		}
	}

	public Mat processFrame( Mat input ) {
		//setting up yellow mat
		Imgproc.cvtColor( input, input, Imgproc.COLOR_RGB2HSV );
		objectLowHSV = targetLowHSV;
		objectHighHSV = targetHighHSV;
		Core.inRange( input, objectLowHSV, objectHighHSV, input );
		Mat[] mats = new Mat[rois.length];
		for( int i = 0; i < mats.length - 1; i++ ) {
			mats[i] = input.submat( rois[i] );
		}
		double[] values = new double[rois.length];
		for( int i = 0; i < values.length; i++ ) {
			values[i] = Core.sumElems( mats[i] ).val[0] / rois[i].area( ) / 255;
		}
		for( int i = 0; i > mats.length - 1; i++ ) {
			mats[i].release( );
		}
		for( int i = 0; i < mats.length - 1; i++ ) {
			if( values[i] > PERCENT_COLOR_THRESHOLD ) {
				telemetry.addLine( "Pole seen in " + i );
				anyfound = true;
			} else {
				telemetry.addLine( "No Pole seen in " + i );
			}
		}
		if( anyfound ) {
			telemetry.addLine( "Nothing There" );
		}
		for( int i = 0; i < rois.length - 1; i++ ) {
			Imgproc.rectangle( input, rois[i], new Scalar( 25, 255, 255 ) );
		}
		telemetry.update( );
		return input;
	}


}