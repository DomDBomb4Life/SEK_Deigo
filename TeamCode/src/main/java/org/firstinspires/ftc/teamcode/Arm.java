package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Arm {
    //declares motors and servos
    private DcMotor liftL, liftR, arm;
    private Servo wrist;
    private OpMode opmode;
    public Arm(OpMode opmode){
        //init OpMode
        this.opmode = opmode;
        //init motors
        liftL = opmode.hardwareMap.get(DcMotor.class, "LiftL");
        liftR = opmode.hardwareMap.get(DcMotor.class, "LiftR");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = opmode.hardwareMap.get(DcMotor.class, "Arm");
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //init the wrist
        wrist = opmode.hardwareMap.get(Servo.class, "Wrist");
    }
    private boolean isMoving(){
        return liftL.isBusy() || liftR.isBusy() || arm.isBusy();
    }
    private final int maxLift = 5500;
    private final int home = 0;
    private final int safety = 50;
    private final int backBoard = -500;
    private enum Location {
        HOME,
        SAFETY,
        TOP,
        BACK_BOARD
    }

    private enum State {
        RAISE,
        LOWER
    }

    private Location location = Location.HOME;
    private State state = State.RAISE;

    public void moveLift() {
        //check if lift is moving
        if(!isMoving()) {
            if(state == State.RAISE) {
                switch(Location) {
                    case HOME:
                    if (opmode.gamepad2.a) {
                        arm.setTargetPosition(safety);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }
                }
            } else if (state == State.LOWER) {

            }
        }
    }
}
