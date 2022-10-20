package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

	Servo leftServo;
	Servo rightServo;

	public Claw ( HardwareMap hw ) {
		this(hw, "left", "right");
	}

	public Claw ( HardwareMap hw, String left, String right ) {
		leftServo = hw.servo.get( left );
		rightServo = hw.servo.get( right );
	}

	public void closeClaw(  ) {
		leftServo.setPosition( .85 );
		rightServo.setPosition( .15 );
	}

	public void openClaw(  ) {
		leftServo.setPosition( .7 );
		rightServo.setPosition( .3 );
	}

}
