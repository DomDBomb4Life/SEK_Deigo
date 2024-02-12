package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ControllerModeV2;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

@Autonomous(name="Auton2023", group="Linear Opmode")
    public class Auton2023 extends LinearOpMode {
        // Enumerations to represent various options
        enum TeamColor { RED, BLUE }
        enum Quadrant { Q1, Q2 }
        enum ParkingPosition { POS_1, POS_2 }
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;




    // Instances of components
        private ControllerModeV2.DriveWheels driveWheels;
        private ControllerModeV2.Lift lift;
        private ControllerModeV2.Plate plate;
        private ControllerModeV2.Gate gate;

        private ControllerModeV2.PlaneLauncher planeLauncher;

        // Variables for user selections
        TeamColor selectedTeam = TeamColor.RED; // default values
        Quadrant selectedQuadrant = Quadrant.Q1; // default values
        ParkingPosition selectedParkingPosition = ParkingPosition.POS_1; // default values
        int delayInSeconds = 0;


        boolean lastA = false;
        boolean lastB = false;
        boolean lastY = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        private AprilTagDetection tagOfInterest = null;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        int spike = 0;

        // UNITS ARE METERS
        double tagsize = 0.166;
        List<Integer> ID_TAGS_OF_INTEREST = new ArrayList<>(); // list of tag IDs of interest


    public void runOpMode() {
        initCamera();
        for (int i = 1; i <= 587; i++) {
            ID_TAGS_OF_INTEREST.add(i);
        }

        driveWheels = new ControllerModeV2.DriveWheels(this);
        plate = new ControllerModeV2.Plate(this);
        lift = new ControllerModeV2.Lift(this, plate);
        gate = new ControllerModeV2.Gate(this);
        planeLauncher = new ControllerModeV2.PlaneLauncher(this);

        while (!isStarted() && !isStopRequested()) {

            //
            gate.controlGate();
            preGameConfig();

            // Use gamepad1 to cycle selections and display on telemetry

            // Telemetry to view selections
            telemetry.addData("Selected Team", selectedTeam);
            telemetry.addData("Selected Quadrant", selectedQuadrant);
            telemetry.addData("Delay (sec)", delayInSeconds);
            telemetry.addData("Parking Position", selectedParkingPosition);
            telemetry.update();


        }
        // Wait for the game to start (after user presses the Start button)
        waitForStart();

        // Detect april tag and determine drive sequences
//        vomitOnSpike(spike);
//        moveToBackdrop(selectedQuadrant, selectedTeam);
//        placePixel(spike);
//        park(selectedParkingPosition, selectedTeam);
        failSafe(selectedQuadrant, selectedTeam, selectedParkingPosition );
        while(opModeIsActive()){}


    }

    public void preGameConfig(){
        // Button A - Cycle Team Color
        if (gamepad2.a && !lastA){
            // Toggle Team Color
            if(selectedTeam == TeamColor.RED) {
                selectedTeam = TeamColor.BLUE;
            } else {
                selectedTeam = TeamColor.RED;
            }
        }
        lastA = gamepad2.a;

        // Button B - Cycle Quadrant
        if (gamepad2.right_bumper && !lastB){
            // Toggle Quadrant
            if(selectedQuadrant == Quadrant.Q1) {
                selectedQuadrant = Quadrant.Q2;
            } else {
                selectedQuadrant = Quadrant.Q1;
            }
        }
        lastB = gamepad2.right_bumper;

        // Button Y - Cycle Parking Position
        if (gamepad2.left_bumper && !lastY){
            // Toggle Parking Position
            if(selectedParkingPosition == ParkingPosition.POS_1) {
                selectedParkingPosition = ParkingPosition.POS_2;
            } else {
                selectedParkingPosition = ParkingPosition.POS_1;
            }
        }
        lastY = gamepad2.left_bumper;

        // DPAD - Adjust Delay Time
        if(gamepad2.dpad_up && !lastDpadUp){
            // Increase delay
            delayInSeconds += 1;
        }
        lastDpadUp = gamepad2.dpad_up;

        if(gamepad2.dpad_down && !lastDpadDown){
            // Decrease delay, don't allow negative values
            delayInSeconds = Math.max(0, delayInSeconds - 1);
        }
        lastDpadDown = gamepad2.dpad_down;

        // Adding a sleep to prevent button mashing, making selection changes more controllable
        sleep(100);
    }

    public void failSafe(Quadrant selectedQuadrant, TeamColor selectedTeam, ParkingPosition selectedParkingPosition) {

        if (selectedQuadrant == Quadrant.Q2) {

            driveWheels.moveLinear(2);
            driveWheels.waitForDrive();

        } else if (selectedQuadrant == Quadrant.Q1) {

            driveWheels.moveLinear(2);
            driveWheels.waitForDrive();

            if (selectedTeam == TeamColor.RED) {
                driveWheels.turn90(1);
                driveWheels.waitForDrive();

            } else if (selectedTeam == TeamColor.BLUE) {
                driveWheels.turn90(-1);
                driveWheels.waitForDrive();

            }
            driveWheels.moveLinear(3.75);
            driveWheels.waitForDrive();
        }
    }

    public void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);

    }

}

/*

I need a new teleop for my autonomous op mode
pseudocode:

This needs to happen before opmode is active:
Determine which team(Red or Blue) I am and which of 2 quadrants I located on the on the field, this should be cycled through with the gamepad 1 dpad up, down, left and right and provide telemetry.
We also need user input to determine how many seconds the delay will be use the gamepad2 dpad up and down
We need the user to determine parking position use gamepad 2 dpad 2
All of the above need tememetry


The 1 april tag will be in one of 3 positions on the feild, we will have a webcam that looks at the field.
For now we just store the tag label that we just saw
We will predefine 3 different x,y coordinate point on the webcam feed and calculate the distance between the center of the april tag and each of the predefined points.
Which ever point has the closest distance, the robot will take one of 1 of 3 different drive sequences(one for each of the points).

(Done) I would like to go create 3 methods in my drive class: moveForward/Backward(tiles), strafe(tiles), turn(degrees)

I would like distance to be defined in tiles: each tile = x encoder steps (The amount of encoder steps for moving forward a tile and strafing a tile are different)
The first point will cause the robot to move forward 1.1 tiles,then strafe left 0.5 tiles , release the pixel, move backwards 0.1 tiles, then strafe right 0.5 tile
The second point will cause the robot to move forward 1.5 tiles, release the pixel, move backwards 0.5 tiles,
The third point will cause the robot to move forward 1.1 tile, strafe right 0.5 tiles, release the pixel, move backwards 0.1 tiles, then strafe left 0.5 tile
All of these methods are independent of the quadrant and they all end in the same spot on the field.
They all use encoders.

Then we need a method to move to the backdrop, this method will need an input of which quadrant we are in, and which team we are on.
The robot will turn 90 degrees (clockwise if on Blue team / counterclockwise if on Red Team)
If in Q1 the robot will wait the delay amount of seconds
The robot will move backwards (3.5 tiles if in Q1 / 1.5 tiles if in Q2)

This is a method to place pixels on the backdrop it will need an input of which point the april tag was classified as
(Strafe left 0.25 tiles if point 1 / do nothing if point 2 / Strafe left 0.25 tiles if point 3)
Move backwards 0.1 tiles
extend lift arms
Call Open Gate Function
retract lift arms
Move forward 0.1 tiles
(Strafe right 0.25 tiles if point 1 / do nothing if point 2 / Strafe left 0.25 tiles if point 3)


Lastly a new method to park: this will input the team we are on and our parking position
strafe left 1 tile if parking position 1 and blue team of parking position 2 and red team
strafe Right 1 tile if parking position 2 and blue team of parking position 1 and red team
move forward 0.5 tiles


*/