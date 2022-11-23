package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RotatingClaw {

	Servo clawServo;
	Servo rotationServo;

	double[] clawPositions;

	public RotatingClaw( HardwareMap hw ) {
		this( hw, "claw", "clawR", new double[]{ 0, 0.5 } );
	}

	public RotatingClaw( HardwareMap hw, String claw, String rotation, double[] clawPositions ) {
		clawServo = hw.servo.get( claw );
		rotationServo = hw.servo.get( rotation );

		this.clawPositions = clawPositions;
	}

	public double[] getPositions( ) {
		return new double[]{ clawServo.getPosition( ), rotationServo.getPosition( ) };
	}

	public void rotateRight() {
		rotate( getRotatePos() - 0.05 );
	}

	public void rotateLeft() {
		rotate( getRotatePos() + 0.05 );
	}

	public void rotate( double pos) {
		rotationServo.setPosition( pos );
	}

	public double getRotatePos() {
		return rotationServo.getPosition();
	}

	public void close( ) {
		clawServo.setPosition( clawPositions[0] );
	}

	public void open( ) {
		clawServo.setPosition( clawPositions[1] );
	}


}
