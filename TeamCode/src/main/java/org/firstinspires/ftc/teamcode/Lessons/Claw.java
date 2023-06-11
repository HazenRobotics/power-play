package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends Lesson{
	double[] limits;
	boolean atLimit;
	Servo servo;
	public Claw( double[] l, Servo s ) {
		limits = l;
		servo = s;
		atLimit = false;
	}
	public void move() {
		if( atLimit ) {
			servo.setPosition( limits[0] );
		} else {
			servo.setPosition( limits[1] );
		}
		atLimit = !atLimit;
	}

	@Override
	void test( ) {
		isTesting = true;
		move();
		move();
		isTesting = false;
	}
}
