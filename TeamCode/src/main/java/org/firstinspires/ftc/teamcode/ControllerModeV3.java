package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive Diego")
public class ControllerModeV3<telemetry> extends LinearOpMode {
   //this is the run function
    @Override
    public void runOpMode() {
        //this connects this to other parts of the code
        DriveTrain Wheels = new DriveTrain(this);
        Claw claw = new Claw(this);
        PlaneLauncher planeLaunch = new PlaneLauncher(this);
        waitForStart();
        while (opModeIsActive()) {
            //this while loop helps the wheels move
            Wheels.drive(this);
            //this helps the claw move
            claw.OpenClose();
            //this helps the launcher move
            planeLaunch.Launch();
        }
    }
}
