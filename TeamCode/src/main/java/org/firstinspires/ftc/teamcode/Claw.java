package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    //This is where important variables are
    Servo claw;
    double clawposition;
    OpMode opMode;
    //constructor for claw
    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.get(Servo.class,"GateServo");
        this.opMode = opMode;
        clawposition = 0;
    }
    //opening and closing claw
    public void OpenClose(){
        if(opMode.gamepad2.b){
            clawposition = 0.32;
        }else{
            clawposition = 0.38;
        }
        claw.setPosition(clawposition);
    }

}
