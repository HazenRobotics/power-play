package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorCalibrator extends OpenCvPipeline {

	Telemetry telemetry;
	Mat calibrationMat = new Mat( );

	String[] colors = {"yellow", "green", "pink"};

	int i = 0;

	double[] colorPercentages = new double[] {0,0,0};
	Scalar[] maxPercentScalars = new Scalar[6];

	static final Rect YELLOW_RECT = new Rect(
			new Point(350, 350 ),
			new Point( 420, 450 ) );
	static final Rect GREEN_RECT = new Rect(
			new Point(590, 350 ),
			new Point( 660, 450 ) );
	static final Rect PINK_RECT = new Rect(
			new Point(800, 350 ),
			new Point( 870, 450 ) );

	public ColorCalibrator( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input ) {

		Imgproc.cvtColor( input, calibrationMat, Imgproc.COLOR_RGB2HSV );


		if (i < 180) {
			Scalar lowHSV = new Scalar( i, 25, 0 );
			Scalar highHSV = new Scalar( i + 15, 255, 255 );


			double yellowPercentage = 0;
			double greenPercentage = 0;
			double pinkPercentage = 0;

			Core.inRange( calibrationMat, lowHSV, highHSV, calibrationMat );

			Mat yellowMat = calibrationMat.submat( YELLOW_RECT );
			Mat greenMat = calibrationMat.submat( GREEN_RECT );
			Mat pinkMat = calibrationMat.submat( PINK_RECT );

			yellowPercentage = Core.sumElems( yellowMat ).val[0] / YELLOW_RECT.area( ) / 255;
			greenPercentage = Core.sumElems( greenMat ).val[0] / GREEN_RECT.area( ) / 255;
			pinkPercentage = Core.sumElems( pinkMat ).val[0] / PINK_RECT.area( ) / 255;


			if( yellowPercentage > colorPercentages[0] ) {
				colorPercentages[0] = yellowPercentage;
				maxPercentScalars[0] = lowHSV;
				maxPercentScalars[1] = highHSV;
			}

			if( greenPercentage > colorPercentages[1] ) {
				colorPercentages[1] = greenPercentage;
				maxPercentScalars[2] = lowHSV;
				maxPercentScalars[3] = highHSV;
			}

			if( pinkPercentage > colorPercentages[2] ) {
				colorPercentages[2] = pinkPercentage;
				maxPercentScalars[4] = lowHSV;
				maxPercentScalars[5] = highHSV;
			}
			i += 1;
		} else {

			int whichMat = (int) ((System.currentTimeMillis() / 1000) % 3);

			Core.inRange( calibrationMat, maxPercentScalars[4], maxPercentScalars[5], calibrationMat );
		}

		Imgproc.rectangle( calibrationMat, YELLOW_RECT, new Scalar (255,0,0) );
		Imgproc.rectangle( calibrationMat, GREEN_RECT, new Scalar (255,0,0) );
		Imgproc.rectangle( calibrationMat, PINK_RECT, new Scalar (255,0,0) );


		return calibrationMat;
	}

	public void waitForTime(long time) {
		long initialTime = System.currentTimeMillis();
		while (System.currentTimeMillis() < initialTime + (time * 1000)){}
	}

	public Scalar[] getMaxPercentScalars() {
		return maxPercentScalars;
	}

	public boolean isFinished() {
		return i <= 180;
	}
}

