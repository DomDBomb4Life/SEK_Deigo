package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain  {
    private DcMotor FrontL, FrontR, BackL, BackR;
    private double MIN_POWER = 0.25;
    private double DEFAULT_POWER = 0.5;
    private double MAX_POWER = 1.0;

    public DriveTrain(LinearOpMode opMode){
        /*This is a constructor function. It tells the robot that there are four dc motors,
        and that two of the motors have the forward direction flipped
        so that when all the wheels go forward, they all go the right way.
         */
        FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
        BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
        FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
        BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");

        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(LinearOpMode opMode) {
        //Update gamepad stick variables
        double leftStickY = opMode.gamepad1.left_stick_y;
        double leftStickX = opMode.gamepad1.left_stick_x;
        double pivot = opMode.gamepad1.right_stick_x;

        //Right trigger pressed robot speeds up. Left trigger pressed robot slows down
        double POWER = DEFAULT_POWER;
        if (opMode.gamepad1.right_trigger > 0){
            POWER = MAX_POWER;
        } else if (opMode.gamepad1.left_trigger > 0){
            POWER = MIN_POWER;
        }
        //Set the power of motors
        double frontRightPower = constrain(-pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;
        double backRightPower = constrain(-pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
        double frontLeftPower = constrain(pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
        double backLeftPower = constrain(pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;
    }
    //constraining method
    private double constrain(double var, double min, double max) {
        var = Math.min(Math.max(var, min), max);
        return var;
    }
    private double constrain(int var, int  min, int max) {
        var = Math.min(Math.max(var, min), max);
        return var;
    }


}
