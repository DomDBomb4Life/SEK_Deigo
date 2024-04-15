package org.firstinspires.ftc.teamcode;

//the import statements import libraries to access information
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain  {
    //this declares motors and variables
    private DcMotor FrontL, FrontR, BackL, BackR;
    private double speed = .5;
    private double leftStickX, leftStickY, pivot;
    public DriveTrain(LinearOpMode opMode){
        //this part tells the code what the motors are
        FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
        BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
        FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
        BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");
        //this flips certain motors so they all move the right direction
        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(LinearOpMode opMode) {
        //this part updates controller
        leftStickY = opMode.gamepad1.left_stick_y;
        leftStickX = opMode.gamepad1.left_stick_x;
        pivot = opMode.gamepad1.right_stick_x;
        updateSpeed(opMode);
        //this parts set power values
        FrontL.setPower((pivot + leftStickX + leftStickY) * speed);
        FrontR.setPower((-pivot + (leftStickY - leftStickX)) * speed);
        BackL.setPower((pivot + (leftStickY - leftStickX)) * speed);
        BackR.setPower((-pivot + leftStickX + leftStickY) * speed);
    }
    //This changes speed when triggers pressed
    private void updateSpeed(LinearOpMode opMode){
        if(opMode.gamepad1.right_trigger == 1){
            speed = 1;
        } else if (opMode.gamepad1.left_trigger == 1) {
            speed = 0.25;
        }else{
            speed = 0.5;
        }
    }
}
