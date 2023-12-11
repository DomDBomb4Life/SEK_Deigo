//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "Drive")
//public class controllerModeOld<telemetry> extends LinearOpMode {
//
//    private DcMotor FrontL;
//    private DcMotor BackL;
//    private DcMotor FrontR;
//    private DcMotor BackR;
//    private DcMotor LiftL;
//    private DcMotor LiftR;
//    private Servo PlaneLaunch;
//    private Servo PlateL;
//    private Servo PlateR;
//    private Servo Gate;
//
//    /**
//     * This function is executed when this Op Mode is selected from the Driver Station.
//     */
//
//
//    @Override
//    public void runOpMode() {
//        float leftStickY;
//        float leftStickX;
//        double driveSpeed;
//        float pivot;
//
//
//
//
//
//
//        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
//        BackL = hardwareMap.get(DcMotor.class, "BackL");
//        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
//        BackR = hardwareMap.get(DcMotor.class, "BackR");
//
//        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackR.setDirection(DcMotorSimple.Direction.REVERSE);
//        waitForStart();
//        if (opModeIsActive()) {
//            //move to init position
//
//
//            while (opModeIsActive()) {
//                // Put run blocks here
//                if (gamepad1.right_trigger == 1) {
//                    driveSpeed = 1.0;
//                }
//                else if(gamepad1.left_trigger == 1){
//                    driveSpeed = 0.25;
//                } else {
//                    driveSpeed = 0.5;
//                }
//
//                leftStickY = -gamepad1.left_stick_y;
//                leftStickX = gamepad1.left_stick_x;
//                pivot = gamepad1.right_stick_x;
//                FrontR.setPower((-pivot + (leftStickY - leftStickX)) * driveSpeed);
//                BackR.setPower((-pivot + leftStickY + leftStickX) * driveSpeed);
//                FrontL.setPower((pivot + leftStickY + leftStickX) * driveSpeed);
//                BackL.setPower((pivot + (leftStickY - leftStickX)) * driveSpeed);
//                telemetry.update();
//            }
//        }
//    }
//
//
//        //constraining methods
//        public double constrain(double Var, double Min, double Max){
//            Var = Math.min(Math.max(Var, Min), Max);
//            return Var;
//        }
//        public int constrain(int Var, int Min, int Max){
//            Var = Math.min(Math.max(Var, Min), Max);
//            return Var;
//        }
//}
//
