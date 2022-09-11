package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO/GAMEPAD TESTER", group = "TeleOp")
public class MassPartTester extends OpMode {
    Servo[] servos = new Servo[12];

    @Override
    public void init() {
        for(int i=0; i<servos.length; i++) {
            servos[i] = hardwareMap.servo.get( "servo"+i );
        }
    }

    @Override
    public void loop() {
        while(gamepad1.right_trigger+gamepad2.right_trigger==2);
        for(int i=0; i<servos.length; i++) {
            servos[i].setPosition(1);
            waitRobot(500);
            servos[i].setPosition(0);
        }

    }
    public void waitRobot( int mills ) {
        long startTime = System.currentTimeMillis( );
        while( (startTime + mills) > System.currentTimeMillis( ) ) {
            telemetry.update( );
        }
    }
}
