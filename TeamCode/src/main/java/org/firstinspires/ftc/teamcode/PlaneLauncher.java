package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {
    Servo launcher;
    double open = 0.5;
    double closed = 0;
    double position = closed;
    boolean isItLaunched = false;
    OpMode opMode;
    //constructor for launcher
    public PlaneLauncher(LinearOpMode opMode){
        launcher = opMode.hardwareMap.get(Servo.class,"launcherServo");
        this.opMode = opMode;
    }
    public void Launch(){
        if(opMode.gamepad2.y && isItLaunched){
            position = closed;
        } else if (opMode.gamepad2.y && !isItLaunched) {
            position = open;
        }
        launcher.setPosition(position);
    }
}
