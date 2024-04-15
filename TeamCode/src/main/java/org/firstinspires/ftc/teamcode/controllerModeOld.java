package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive")
public class controllerModeOld<telemetry> extends LinearOpMode {
    //this is where the variables are
DriveTrain Wheels = new DriveTrain(this);
Claw Servoclaw = new Claw(this);
    @Override
    public void runOpMode() {
        while (opModeIsActive()) {
            Wheels.drive(this);
            Servoclaw.OpenClose();
        }
    }
}
