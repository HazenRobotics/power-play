package org.firstinspires.ftc.teamcode.vision;

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
	int j = 0;

	double[] colorPercentages = new double[3];
	Scalar[] maxPercentScalars = new Scalar[6];
	double maxPercent = 0;

	static final Rect ROI = new Rect(
			new Point( 320, 180 ),
			new Point( 960, 540 ) );

	static double PERCENT_COLOR_THRESHOLD = 0.02;

	public ColorCalibrator( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input ) {
		Scalar lowHSV = new Scalar( j, 25, 0 );
		Scalar highHSV = new Scalar( j + 10, 255, 255 );

		Imgproc.cvtColor( input, calibrationMat, Imgproc.COLOR_RGB2HSV );

		double percentage = 0;

		Core.inRange( calibrationMat, lowHSV, highHSV, calibrationMat );

		percentage = Core.sumElems( calibrationMat ).val[0] / ROI.area( ) / 255;

		if (percentage > maxPercent) {
			maxPercent = percentage;
			maxPercentScalars[i] = lowHSV;
			maxPercentScalars[i + 1] = highHSV;
		}

		if (j % 180 == 0) {
			colorPercentages[i] = maxPercent;
			telemetry.addLine(colors[i] + " done");
			telemetry.addData("Best Percent", maxPercent );
			telemetry.addData("Low", maxPercentScalars[i] );
			telemetry.addData("High", maxPercentScalars[i + 1] );
			telemetry.addLine("Reorient cone and wait 10 seconds");
			telemetry.update();
			waitForTime( 10 );
		}

		j += 10;
		i = j / 180;

		return calibrationMat;
	}

	public void waitForTime(long time) {
		long initialTime = System.currentTimeMillis();
		while (System.currentTimeMillis() < initialTime + (time * 1000)){}
	}
}

