package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.Drive;

public class CameraAngler {

	public Servo servo;

	public double servoMin;
	public double servoMax;

	public double angleMin;
	public double angleMax;

	public CameraAngler (HardwareMap hw) {
		this(hw, "angler", 0, 0.5, 0, 90);
	}

	public CameraAngler( HardwareMap hw, String name, double servoMin, double servoMax, double angleMin, double angleMax) {
		servo = hw.servo.get( name );
		this.servoMin = servoMin;
		this.servoMax = servoMax;
		this.angleMin = angleMin;
		this.angleMax = angleMax;
	}

	public void pointCameraToPosition( double x, double y ) {
		servo.setPosition( angleToServo( Math.toDegrees( Math.atan( y/x ) ) ) );
	}

	public double angleToServo( double angle ){
		return Drive.normalize( angle, angleMin, angleMax, servoMin, servoMax);
	}

	public double servoToAngle( double pos ){
		return Drive.normalize( pos, servoMin, servoMax, angleMin, angleMax);
	}

}
