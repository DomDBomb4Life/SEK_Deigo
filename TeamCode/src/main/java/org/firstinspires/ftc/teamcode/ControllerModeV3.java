package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class ControllerModeV3<telemetry> extends LinearOpMode {
DriveTrain Wheels = new DriveTrain(this);
    @Override
    public void runOpMode() {
        while (opModeIsActive()) {
            Wheels.drive(this);
        }
    }
}
