package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ConnersArm {
	Servo arm;
	public ConnersArm( HardwareMap map ){
		arm = map.get(Servo.class,"arm1");
		armUp();
	}

	public void armUp(){
		arm.setPosition( 0.5 );
	}

	public void armDown(){
		arm.setPosition( 1 );
	}

	public void move(){
		if (arm.getPosition() == 0.5)
			armDown();
		else
			armUp();
	}


}
