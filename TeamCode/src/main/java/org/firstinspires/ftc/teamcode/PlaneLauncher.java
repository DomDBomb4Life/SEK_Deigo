package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {
    Servo launcher;
    double open = 0.5;
    double closed = 0;
    boolean position = false;
    boolean isItLaunched = false;
    OpMode opMode;
    //constructor for launcher
    public PlaneLauncher(LinearOpMode opMode){
        launcher = opMode.hardwareMap.get(Servo.class,"LauncherServo");
        this.opMode = opMode;
        launcher.setPosition(0);
    }
    public void Launch() {
        //if(opMode.gamepad2.y && isItLaunched){
        //position = closed;
        //isItLaunched = false;
        //} else if (opMode.gamepad2.y && !isItLaunched) {
        //position = open;
        //isItLaunched = true;

        //launcher.setPosition(position);

        if (opMode.gamepad2.y && !isItLaunched) {
            position = !position; //toggle the state
            isItLaunched = true;
            launcher.setPosition(0.5);
        } else if (!opMode.gamepad2.y) {
            isItLaunched = false; //button released, can be pressed again

        }
        if (position) {
            launcher.setPosition(0.5);
            opMode.telemetry.addLine("Plane launched");
        } else {
            launcher.setPosition(0);
            opMode.telemetry.addLine("Plane Docked");
        }
    }
}

