package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipelines.BarcodePositionDetector;
import org.firstinspires.ftc.teamcode.vision.pipelines.ColorCalibrator;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class ColorCalibratorUtil {

	Telemetry telemetry;
	private OpenCvWebcam webcam;
	private ColorCalibrator pipeline;

	public ColorCalibratorUtil( HardwareMap hardwareMap, String webcamName, Telemetry telemetry ) {
		this.telemetry = telemetry;
		setup( hardwareMap, webcamName );
	}

	public void setup( HardwareMap hardwareMap, String webcamName ) {

		int cameraMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) );
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hardwareMap.get( WebcamName.class, webcamName ), cameraMonitorViewId );
		pipeline = new ColorCalibrator( telemetry );
		webcam.setPipeline( pipeline );
		FtcDashboard.getInstance().startCameraStream( webcam, 60 );
	}

	public void init( ) {
		openCameraDevice( );
	}

	public void setTimeoutTime( int milliseconds ) {
		// Timeout for obtaining permission is configurable. Set before opening.
		webcam.setMillisecondsPermissionTimeout( milliseconds );
	}

	public void openCameraDevice( ) {

		webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
			@Override
			public void onOpened( ) {
				webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
			}

			@Override
			public void onError( int errorCode ) {
				//This will be called if the camera could not be opened
				Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
			}
		} );
	}

	public boolean isFinished( ) {
		return pipeline.isFinished( );
	}

	public Scalar[] getScalars( ) {
		return pipeline.getMaxPercentScalars( );
	}

	public void stopCamera( ) {
		webcam.stopStreaming( );
	}
}
