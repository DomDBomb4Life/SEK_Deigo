package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain  {
    private DcMotor FrontL, FrontR, BackL, BackR;

    public DriveTrain(LinearOpMode opMode){
        FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
        BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
        FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
        BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");

        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(LinearOpMode opMode) {
        double leftStickY = opMode.gamepad1.left_stick_y;
        double leftStickX = opMode.gamepad1.left_stick_x;
        double pivot = opMode.gamepad1.right_stick_x;
    }
}
