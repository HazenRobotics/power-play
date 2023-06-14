package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MegansClaw {
	Servo claw;
	public MegansClaw( HardwareMap map ){
		claw = map.get(Servo.class,"claw");

	}
	public void open(){}
	public void close(){}
	public void motion(){
//		if (){
//
//		}
//		else{
//
//		}
	}

}
