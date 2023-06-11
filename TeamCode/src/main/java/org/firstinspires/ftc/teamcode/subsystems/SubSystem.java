package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public abstract class SubSystem {
	HardwareMap hw;
	public SubSystem(HardwareMap h) {
		hw=h;
	}
	// Returns all important data from subsystem
	abstract ArrayList<Object> getData();
	//test out system
	abstract void test();


}
