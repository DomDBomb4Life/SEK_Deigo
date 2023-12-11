package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Drive")
public class ControllerModeV2 extends LinearOpMode {

    // Class for Drive Wheels
    public static class DriveWheels {
        private DcMotor FrontL, FrontR, BackL, BackR;
        private LinearOpMode opMode; // To access gamepad inputs and telemetry from the main op mode
        //private UltrasonicSensor distanceSensor; // Sensor for detecting distance

        // Constants for distance-based speed adjustment
        private final double MIN_POWER = 0.25;
        private final double DEFAULT_POWER = 0.5;
        private final double MAX_POWER = 1.0;

        private final double DECELERATION_START_DISTANCE = 1000; // Placeholder value, need to be experimentally determined
        private final double DECELERATION_END_DISTANCE = 200; // Placeholder value, need to be experimentally determined


        // Tile size (23 inches) divided by inches per encoder step gives the encoder steps per tile
        double encoderStepsPerTileLinear = 1025;

        double encoderStepsPerTileStrafe = 3330;

        double encoderStepsPer90Deg = 800;
        double autonMotorPower = 0.5;

        // Constructor
        public DriveWheels(LinearOpMode opMode) {
            this.opMode = opMode;

            // Motor initialization
            FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
            BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
            FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
            BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");

            // Set motor directions
            FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
            BackR.setDirection(DcMotorSimple.Direction.REVERSE);

            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        // Method to control the drive based on gamepad input
        public void drive() {
            double leftStickY = -opMode.gamepad1.left_stick_y;
            double leftStickX = opMode.gamepad1.left_stick_x;
            double pivot = opMode.gamepad1.right_stick_x;

            if (opMode.gamepad1.left_bumper){
                leftStickY = opMode.gamepad1.left_stick_y;
                leftStickX = -opMode.gamepad1.left_stick_x;
                pivot = -opMode.gamepad1.right_stick_x;
            }

            double POWER = DEFAULT_POWER;

            // Apply triggers for manual speed override
            if (opMode.gamepad1.right_trigger > 0) {
                POWER = MAX_POWER;
            } else if (opMode.gamepad1.left_trigger > 0) {
                POWER = MIN_POWER;
            }

            // Set motor powers
            double frontRightPower = constrain(-pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;
            double backRightPower = constrain(-pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
            double frontLeftPower = constrain(pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
            double backLeftPower = constrain(pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;

            // Set motor powers
            FrontL.setPower(frontLeftPower);
            FrontR.setPower(frontRightPower);
            BackL.setPower(backLeftPower);
            BackR.setPower(backRightPower);
            // Telemetry for debugging

            opMode.telemetry.addData("FrontL", FrontL.getCurrentPosition());
            opMode.telemetry.addData("FrontR", FrontR.getCurrentPosition());
            opMode.telemetry.addData("BackL", BackL.getCurrentPosition());
            opMode.telemetry.addData("BackR", BackR.getCurrentPosition());
        }

        // Constraining method
        private double constrain(double var, double min, double max) {
            var = Math.min(Math.max(var, min), max);
            return var;
        }

        public void moveLinear(double tiles) {
            int encoderSteps = (int) (tiles * encoderStepsPerTileLinear);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setPower(autonMotorPower);
            FrontR.setPower(autonMotorPower);
            BackL.setPower(autonMotorPower);
            BackR.setPower(autonMotorPower);
            FrontR.setTargetPosition(encoderSteps);
            FrontL.setTargetPosition(encoderSteps);
            BackR.setTargetPosition(encoderSteps);
            BackL.setTargetPosition(encoderSteps);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void strafe(double tiles) {
            int encoderSteps = (int) (tiles * encoderStepsPerTileStrafe);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setPower(autonMotorPower);
            FrontR.setPower(autonMotorPower);
            BackL.setPower(autonMotorPower);
            BackR.setPower(autonMotorPower);
            FrontR.setTargetPosition(-encoderSteps);
            FrontL.setTargetPosition(encoderSteps);
            BackR.setTargetPosition(encoderSteps);
            BackL.setTargetPosition(-encoderSteps);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        public void turn90(int direction) {
            int encoderSteps = (int) (direction * encoderStepsPer90Deg);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setPower(autonMotorPower);
            FrontR.setPower(autonMotorPower);
            BackL.setPower(autonMotorPower);
            BackR.setPower(autonMotorPower);
            FrontR.setTargetPosition(encoderSteps);
            FrontL.setTargetPosition(-encoderSteps);
            BackR.setTargetPosition(encoderSteps);
            BackL.setTargetPosition(-encoderSteps);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void waitForDrive() {
            while (BackL.isBusy() || BackR.isBusy() || FrontR.isBusy() || FrontL.isBusy()) {
                opMode.telemetry.addData("FrontL", FrontL.getCurrentPosition());
                opMode.telemetry.addData("FrontR", FrontR.getCurrentPosition());
                opMode.telemetry.addData("BackL", BackL.getCurrentPosition());
                opMode.telemetry.addData("BackR", BackR.getCurrentPosition());
                opMode.telemetry.update();
            }

        }
    }
    // Class for the Lift

    public static class Lift {
        private double liftPower = 1.0;
        private int maxLiftHeight = 5500;
        private int minLiftHeight = 0;
        private double SPEED;
        private DcMotor LiftL, LiftR;
        private Plate plate; // Reference to Plate object
        private LinearOpMode opMode;
        private int liftHeight = 0;
        private boolean homestate = false;
        private boolean waitingOnLift = false;
        private boolean phase2 = false;
        private boolean buttonPressed = false;

        public Lift(LinearOpMode opMode, Plate plate) {
            this.opMode = opMode;
            this.plate = plate;
            this.LiftL = this.opMode.hardwareMap.get(DcMotor.class, "LiftL");
            this.LiftR = this.opMode.hardwareMap.get(DcMotor.class, "LiftR");
            LiftR.setDirection(DcMotorSimple.Direction.REVERSE);
            ;
            LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LiftR.setPower(liftPower);
            LiftL.setPower(liftPower);
            liftHeight = minLiftHeight;
            LiftR.setTargetPosition(liftHeight);
            LiftL.setTargetPosition(liftHeight);
            LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public double constrain(double var, double min, double max) {
            var = Math.min(Math.max(var, min), max);
            return var;
        }

        public void controlLift() {
            boolean plateMoving = false;
            if (opMode.gamepad2.a && !buttonPressed && !plate.calledToMove) {
                homestate = !homestate;// Toggle the state only when the button is newly pressed
                buttonPressed = true;
                plate.calledToMove = true;

            } else if (!opMode.gamepad2.x && buttonPressed) {
                buttonPressed = false; // Reset the flag when the button is released
            } else {
            }
            if (plate.calledToMove) {
                if (homestate) {
                    plateMoving = plate.controlPlate(true, phase2);
                    if (!plateMoving && !phase2) {
                        if (!waitingOnLift) {
                            raiseLift();
                        }
                    }
                } else {
                    plateMoving = plate.controlPlate(false, phase2);
                    if (!plateMoving && !phase2) {
                        if (!waitingOnLift) {
                            lowerLift();
                        }
                    }
                }
                if (waitingOnLift && !LiftL.isBusy() && !LiftR.isBusy() && !phase2) {
                    waitingOnLift = false;
                    phase2 = true;
                }
                if (homestate) {
                    plateMoving = plate.controlPlate(true, phase2);
                } else {
                    plateMoving = plate.controlPlate(false, phase2);

                }
                if (!plateMoving && phase2) {
                    plate.calledToMove = false;
                    phase2 = false;
                }
            }
/*
            if (opMode.gamepad2.right_trigger>0 ) {
                if (!(opMode.gamepad2.left_trigger>0))
                plate.liftArm();
                if(opMode.gamepad2.left_trigger>0){
                    lowerLift();
                    opMode.sleep(300);
                    plate.lowerArm();
                }
            }
*/
            opMode.telemetry.addData("plateMoving", plateMoving);
            opMode.telemetry.addData("phase2", phase2);
            opMode.telemetry.addData("liftHeight", liftHeight);
            opMode.telemetry.addData("waitingOnLift", waitingOnLift);
            opMode.telemetry.addData("Lift Position", LiftL.getCurrentPosition());

        }

        public void waitForLift() {
            while (LiftL.isBusy() || LiftR.isBusy()) {
            }
        }

        public void raiseLift() {
            liftHeight = maxLiftHeight;
            LiftL.setTargetPosition(liftHeight);
            LiftR.setTargetPosition(liftHeight);
            LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitingOnLift = true;
        }

        public void lowerLift() {
            LiftL.setTargetPosition(minLiftHeight);
            LiftR.setTargetPosition(minLiftHeight);
            LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitingOnLift = true;
        }

    }

    public static class Plate {
        private Servo Wrist;
        private DcMotor Arm;
        private LinearOpMode opMode;
        private final double posToDegArm = 360.0 / 1425.1;
        private final double degToPosWrist = 1.0 / 300.0;
        private final double armDegToWristDeg = -150.0 / 360.0;
        private double armPosToWristPos = 0.002245456;//posToDegArm*armDegToWristDeg*degToPosWrist;
        private final double wristHome = 0.53;
        private final double wristSafetyPos = 0.63;
        private final double wristBackdropPos = 0.2499999;

        private final int armHome = 10;
        private final int armSafteyPos = 64;
//        private final int armBackdropPos = 0;
        private final int armBackdropPos = -500;
        private int armSpeed;
        private double armPower = 1;

        private boolean toggle = true;
        private boolean buttonPressed = false;

        private final double POSITION_THRESHOLD = 0.05;
        private double wristPos;
        private int armPos;
        private boolean movingToFromSafety = false;
        private boolean movingToFromBackdrop = false;
        private boolean moving = false;
        public boolean calledToMove = false;

        // Constructor
        public Plate(LinearOpMode opMode) {
            this.opMode = opMode;

            // Initialize servos
            Wrist = opMode.hardwareMap.get(Servo.class, "Wrist");
            Arm = opMode.hardwareMap.get(DcMotor.class, "Arm");
            Arm.setDirection(DcMotorSimple.Direction.REVERSE);
            ;
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setPower(armPower);
            init();

        }
        public void liftArm(){
            Arm.setTargetPosition(-700);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void lowerArm(){
            Arm.setTargetPosition(-400);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void init() {
            armPos =armHome;
            Arm.setTargetPosition(armPos);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristPos = wristHome;
            Wrist.setPosition(wristPos);

        }

        private boolean wristMoving() {
            if (Math.abs(Wrist.getPosition() - wristPos) < POSITION_THRESHOLD) {
                return false;
            }
            return true;
        }

        public void moveToSafety(boolean homeToSafety) {
            armSpeed = 30;
            if (homeToSafety) {
                if ((armPos != armSafteyPos) && movingToFromSafety) {
                    if (!(Arm.isBusy() || wristMoving())) {
                        armPos += armSpeed;
                        armPos = Math.min(armPos, armSafteyPos);
                        Arm.setTargetPosition(armPos);
                        wristPos += armSpeed * armPosToWristPos;
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Wrist.setPosition(wristPos);
                    }
                } else if (armPos == armSafteyPos && !(Arm.isBusy() || wristMoving())) {
                    movingToFromSafety = false;
                }
            } else {
                armSpeed = -armSpeed;
                if ((armPos != armHome) && movingToFromSafety) {
                    if (!(Arm.isBusy() || wristMoving())) {
                        armPos += armSpeed;
                        armPos = Math.max(armPos, armHome);
                        Arm.setTargetPosition(armPos);
                        wristPos += armSpeed * armPosToWristPos;
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Wrist.setPosition(wristPos);
                    }
                } else if (armPos == armHome && !(Arm.isBusy() || wristMoving())) {
                    movingToFromSafety = false;
                }
            }
        }

        public void moveToBackdrop(boolean safetyToBack) {
            Arm.setPower(armPower * 0.5);
            if (safetyToBack) {
                if ((armPos != armBackdropPos) && movingToFromBackdrop) {
                    if (!(Arm.isBusy() || wristMoving())) {
                        armPos = armBackdropPos;
                        wristPos = wristBackdropPos;
                        Arm.setTargetPosition(armPos);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Wrist.setPosition(wristPos);
                    }
                } else if (armPos == armBackdropPos && !(Arm.isBusy() || wristMoving())) {
                    movingToFromBackdrop = false;
                    Arm.setPower(armPower * 1);

                }
            } else {
                if ((armPos != armSafteyPos) && movingToFromBackdrop) {
                    if (!(Arm.isBusy() || wristMoving())) {
                        armPos = armSafteyPos;
                        Arm.setTargetPosition(armPos);
                        wristPos = wristSafetyPos;
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Wrist.setPosition(wristPos);
                    }
                } else if (armPos == armSafteyPos && !(Arm.isBusy() || wristMoving())) {
                    movingToFromBackdrop = false;
                }
            }
        }

        public boolean controlPlate(boolean up, boolean phase2) {

            if (calledToMove && !phase2) {
                moving = true;
                if (up) {
                    movingToFromSafety = true;
                } else {
                    movingToFromBackdrop = true;
                }
            }
            moveToSafety(up);
            moveToBackdrop(up);
            if (!phase2 && ((up && !movingToFromSafety) || (!up && !movingToFromBackdrop)) && moving) {
                moving = false;
            }
            if (calledToMove && phase2) {
                moving = true;
                if (up) {
                    movingToFromBackdrop = true;
                } else {
                    movingToFromSafety = true;
                }
            }
            moveToSafety(up);
            moveToBackdrop(up);
            if (phase2 && ((!up && !movingToFromSafety) || (up && !movingToFromBackdrop)) && moving) {
                moving = false;
                calledToMove = false;
            }

            return moving;
        }

        public void plateTelemetry() {
            opMode.telemetry.addData("Wrist Position: ", wristPos);
            opMode.telemetry.addData("Arm Position: ", Arm.getCurrentPosition());
            opMode.telemetry.addData("calledToMove: ", calledToMove);
            opMode.telemetry.addData("movingToFromSafety: ", movingToFromSafety);
            opMode.telemetry.addData("movingToFromBackdrop: ", movingToFromBackdrop);


        }

        // Manual adjustment of the Plate servos
        public void adjustServosManually() {
            if (opMode.gamepad2.dpad_right) {
                wristPos += 0.01; // Increment for adjustment
            } else if (opMode.gamepad2.dpad_left) {
                wristPos -= 0.01; // Increment for adjustment\
            }
            opMode.sleep(100);

            // Constrain servo positions between 0.0 and 1.0
            wristPos = Math.max(0.0, Math.min(1.0, wristPos));

            // Update servo positions
            Wrist.setPosition(wristPos);


            // Telemetry to show current servo positions
            opMode.telemetry.addData("wrist Position", wristPos);


        }
        // Additional logic to call when plate movement has concluded
        /*
        public void getPos(){
            double PlateLPosition = PlateL.getPosition();
            double PlateRPosition = PlateR.getPosition();
            opMode.telemetry.addData("PlateL Position", PlateLPosition);
            opMode.telemetry.addData("PlateR Position", PlateRPosition);
        }
*/
    }

    // Class for the Gate

    public static class Gate {
        private final LinearOpMode opMode;
        private Servo gateServo;
        private boolean gateToggle = true;
        private boolean gateButtonPressed = false;
        private double gatePos;
        private double close = 0.05;
        private double open = 0;

        public boolean gateObstructed = false;
        private boolean isAttemptingToClose = false;
        private double attemptStartTime;
        private boolean hasBeenOpenedForRetry = false;

        public Gate(LinearOpMode opMode) {
            // Initialize servo
            this.opMode = opMode;
            gateServo = opMode.hardwareMap.get(Servo.class, "GateServo");
            closeGate();
        }
        public void controlGate() {
            opMode.telemetry.addData("time", opMode.time);
            /*
            if (opMode.gamepad2.b && !gateButtonPressed) {
                gateToggle = !gateToggle; // Toggle the state
                gateButtonPressed = true;
            } else if (!opMode.gamepad2.b) {
                gateButtonPressed = false; // Button has been released, can be pressed again.
            }
            */
            if (!opMode.gamepad2.b) {
                closeGate();
                opMode.telemetry.addLine("Gate Closed");
            } else {
                openGate();
                opMode.telemetry.addLine("Gate Open");
            }
        }

        /*
        public void controlGate() {
            opMode.telemetry.addData("time", opMode.time);
            if (opMode.gamepad2.b && !gateButtonPressed) {
                gateToggle = !gateToggle; // Toggle the state
                gateButtonPressed = true;
            } else if (!opMode.gamepad2.b) {
                gateButtonPressed = false; // Button has been released, can be pressed again.
            }
            if (gateToggle) {
                closeGate();
                gateObstructed = isGateObstructed();
                if (!gateObstructed) {
                    opMode.telemetry.addLine("Gate Closed");
                }
            } else {
                openGate();
                gateObstructed = true;
                opMode.telemetry.addLine("Gate Open");
            }

        }
        */

        public void openGate() {
            gatePos = open;
            gateServo.setPosition(gatePos);
            /*
            isAttemptingToClose = false; // reset the close attempt flag
            hasBeenOpenedForRetry = true; // indicate that gate has been opened for retry
            attemptStartTime = opMode.time; // reset the attempt start time
            */

        }

        public void closeGate() {
            gatePos = close;
            gateServo.setPosition(gatePos);
        }
        /*
        public void closeGate() {
            if (!isAttemptingToClose) {
                // Starting the close attempt now
                gatePos = close;
                gateServo.setPosition(gatePos);
                attemptStartTime = opMode.time;
                isAttemptingToClose = true; // we are now attempting to close gate
                hasBeenOpenedForRetry = false; // reset the retry flag
            } else {
                // We are in the process of a close attempt
                if (opMode.time - attemptStartTime < 1.0) {
                    // We are within the first second of attempting to close.
                    if (isGateObstructed()) {
                        // Gate is obstructed, so try to reopen for retry
                        openGate();
                    }
                } else {
                    // It has been at least one second since we started trying to close
                    isAttemptingToClose = false;
                    if (isGateObstructed() && !hasBeenOpenedForRetry) {
                        // If the gate is still obstructed and we haven't yet tried the open-for-retry, do so now
                        openGate();
                    }
                }
            }
            */

        /*
        private boolean isGateObstructed(){
        if (Math.abs(gateServo.getPosition()-close)>0.0016) {
            opMode.telemetry.addLine("Gate Obstructed");
            return false;
        }
        return true;
        }
        */
    }

    public static class LinearGeckoWheels {
        private DcMotor geckoMotor;
        private LinearOpMode opMode; // To access gamepad inputs and telemetry from the main op mode
        private boolean isRunning = true; // Default is running
        private boolean buttonPressed = false; // To track the current state of the button press
        private boolean toggle = false;
        private double wheelPower = 1.0;
        // Constructor
        public LinearGeckoWheels(LinearOpMode opMode) {
            this.opMode = opMode;

            // Initialize motor
           geckoMotor = opMode.hardwareMap.get(DcMotor.class, "GeckoMotor");
        }


        // Method to update the state of the gecko wheels based on gamepad input
        public void updateGeckoWheels() {
            // This is a state-change detection - when the button is pressed, but was not pressed before
            if (opMode.gamepad2.x && !buttonPressed) {
                toggle = !toggle ;// Toggle the state only when the button is newly pressed
                buttonPressed = true;
            } else if (!opMode.gamepad2.x && buttonPressed) {
                buttonPressed = false; // Reset the flag when the button is released
            }
            if (toggle){
                geckoMotor.setPower(wheelPower);

            } else {
                ejectPixel(true);
            }
        }

        public void ejectPixel(boolean Fast) {
            if (Fast){
                geckoMotor.setPower(-wheelPower);
            } else {
                geckoMotor.setPower(-wheelPower);
            }
        }

    }

    // Class for the Plane Launcher
    public static class PlaneLauncher {
        private final LinearOpMode opMode;
        private Servo launcherServo;
        private boolean planeToggle = false;
        private boolean planeButtonPressed = false;
        private double close = 0.1;
        private double open = 0.1;


        public PlaneLauncher(LinearOpMode opMode) {
            // Initialize servo
            this.opMode = opMode;
            launcherServo = opMode.hardwareMap.get(Servo.class, "LauncherServo");
            launcherServo.setPosition(0);
        }

        /**
         * Toggles the plane launcher servo 90 degrees back and forth with the A button on gamepad 2.
         */
        public void controlLauncher() {
            if (opMode.gamepad2.y && !planeButtonPressed) {
                planeToggle = !planeToggle; // Toggle the state
                planeButtonPressed = true;
            } else if (!opMode.gamepad2.y) {
                planeButtonPressed = false; // Button has been released, can be pressed again.
            }
            if (planeToggle) {
                launcherServo.setPosition(1.0);
                opMode.telemetry.addLine("Plane Launched");
            }else{
                launcherServo.setPosition(0);
                opMode.telemetry.addLine(" Plane Docked");
            }
        }
    }
    // Instances of components
    private DriveWheels driveWheels;
    private Lift lift;
    private Plate plate;
    private Gate gate;
    private LinearGeckoWheels geckoWheels;
    private PlaneLauncher planeLauncher;
    @Override
        public void runOpMode() {
            // Hardware initialization

                    //hardwareMap.get(UltrasonicSensor.class, "distanceSensor");

            // Initialize the components using the class instance variables
            driveWheels = new DriveWheels(this);
            plate = new Plate(this);
            lift = new Lift(this, plate);
            gate = new Gate(this);
            planeLauncher = new PlaneLauncher(this);
            geckoWheels = new LinearGeckoWheels(this); // Make sure geckoWheels is a class instance variable

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive()) {
                // Update the state of the drive wheels
                driveWheels.drive();
                // Update the state of the lift
                lift.controlLift();//Uses Gamepad 2 a for raise/lower Lift  and dpad down for lifting robot of field
                gate.controlGate(); // Uses Gamepad 2 b
              plate.adjustServosManually();
                geckoWheels.updateGeckoWheels();//Use Gamepad 2 x
                planeLauncher.controlLauncher();//Use Gamepad 2 y

                plate.plateTelemetry();


                // Telemetry update. You can add more telemetry outputs as needed for debugging.
//                telemetry.addData("Lift Position", lift.getLiftPosition()); // Assuming getLiftPosition exists and returns some value
//                telemetry.addData("Plate Horizontal", plate.isPlateHorizontal()); // Assuming isPlateHorizontal method exists
                telemetry.update();
            }
        }
    }