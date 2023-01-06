package org.firstinspires.ftc.teamcode.Auton;


import static org.firstinspires.ftc.teamcode.Auton.AprilTagImageRecognition.FEET_PER_METER;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Autonomous Right Stack", group="Robot")

public class AutonRightStack extends IterativeAuton{
    public double LIFTMOTORPOWER = 1.0;

    @Override
    public void start() {
        hasStarted = true;
        runtime.reset();
        int id = getTag();
        pathWithoutCamera();
        robot.flipToPosition(0.5);

        // path done
//        robot.setLift(10560, LIFTMOTORPOWER);
//        robot.flipL.setPosition(0);
//        robot.flipR.setPosition(1); // flip
//        while(robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() < 2 * robot.minHeightForFlip) {}
//        robot.rotate.setPosition(0); // rotate
//        robot.claw.setPosition(0); // open
    }
    public void pathWithoutCamera() {
        robot.claw.setPosition(1); // close claw
        moveConeToHighTerminal();
        for(int numTimes = 4; numTimes > 3; numTimes--) { // change to numTimes >= 0 to do all five cones
            moveToStack(numTimes);
            moveBackToHighTerminal();
        }
    }

    public void moveBackToHighTerminal() {
        robot.claw.setPosition(1);
        robot.setLift(10560,LIFTMOTORPOWER);
        boolean dontFlip = true;
        sleep(100);
        // TODO: This the movement back from stack to terminal, should be opposite of moveToStack encoder functions
        encoderDriveNoWaiting(0.3,180,48,3);
        boolean dontStop = true;
        robot.flipToPosition(1); // flip

        while(dontStop || dontFlip){
            dontStop = robot.fLMotor.isBusy() || robot.fRMotor.isBusy() || robot.bLMotor.isBusy() || robot.bRMotor.isBusy();
            dontFlip = robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() < 2 * robot.minHeightForFlip;
            if(!dontStop) {
                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(!dontFlip){
                robot.rotate.setPosition(0); // rotate
                robot.claw.setPosition(0); // open
            }
        }
        // TODO: strafe back to next to terminal
        encoderDrive(0.3,90, 12,1);
        // TODO: Move until cone is on top of high terminal
        encoderDrive(0.5,0,6,1);
        robot.claw.setPosition(0);

    }

    public void moveToStack(int numTimes) {
        // TODO: move out of the way into center of tile
        encoderDrive(0.3,270, 12,1);
        //move lift back down
        robot.setLift(numTimes * 440,LIFTMOTORPOWER); // stack cone #5
        robot.claw.setPosition(1);
        robot.rotate.setPosition(1);
        // TODO: begin moving toward stack, inches need to get robot to wall stack
        encoderDriveNoWaiting(0.3,0,48,3);
        //prepare claw to pick up cones
        sleep(250);
        robot.flipToPosition(0); // back to normal flipped state
        sleep(250);
        robot.claw.setPosition(0);
        while(robot.fLMotor.isBusy() || robot.fRMotor.isBusy() || robot.bLMotor.isBusy() || robot.bRMotor.isBusy()){
            // stall
        }
        // Stop all motion;
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Move until cone is on top of high terminal
        encoderDrive(0.5,180,6,1);
    }
    public void moveConeToHighTerminal(){
        robot.setLift(10560,LIFTMOTORPOWER);
        boolean dontFlip = true;
        robot.flipToPosition(1); // flip
        // TODO: initial movement, should get robot to right next to the high terminal
        //  (robot should be positioned with claw facing right, centered in the tile)
        encoderDriveNoWaiting(0.3,90,82,2);
        boolean dontStop = true;

        while(dontStop || dontFlip){
            dontStop = robot.fLMotor.isBusy() || robot.fRMotor.isBusy() || robot.bLMotor.isBusy() || robot.bRMotor.isBusy();
            dontFlip = robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() < 2 * robot.minHeightForFlip;
            if(!dontStop) {
                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(!dontFlip){
                robot.rotate.setPosition(0); // rotate
                robot.claw.setPosition(0); // open
            }
        }
        // TODO: Move until cone is on top of high terminal
        encoderDrive(0.5,0,6,1);
        robot.claw.setPosition(0); // open claw
        sleep(250);
    }
    public void pathWithCamera() {
        telemetry.addData("Message: ", robot.message);
        telemetry.update();
        sleep(1000);
        finalMessage = robot.message;

        robot.claw.setPosition(1);

        encoderDrive(0.5,90,29, 2);
        if(finalMessage == null){
            finalMessage = robot.message;
        }
        if(finalMessage != null){
            switch (finalMessage) {
                case "https://left.com":
                    // strafe left one tile
                    encoderDrive(0.5, 0, 24, 1.5);
                    break;
                case "https://middle.com":
                    // no need to move
                    break;
                case "https://right.com":
                    // strafe right one tile
                    encoderDrive(0.5, 180, 24, 1.5);
                    break;
                default:
                    // hope it's middle, attempting to recheck
                    telemetry.addData("We got used by another team :(", "unfortunate L");
                    telemetry.update();
            }
        }
    }

    public int getTag() {
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 2.49539580e+03;
        double fy = 2.49845758e+03;
        double cx = 6.14724848e+02;
        double cy = 3.40242180e+02;

        // UNITS ARE METERS
        double tagsize = 0.04;

        int left = 1; // Tag ID 1 from the 36h11 family for left movement
        int middle = 2; // Tag ID 2 from the 36h11 family for middle movement
        int right = 3; // Tag ID 3 from the 36h11 family for right movement

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        ElapsedTime runtime = new ElapsedTime();
        double currentTime = runtime.milliseconds();
        while (runtime.milliseconds() - currentTime < 5000) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            boolean tagFound = false;
            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine(String.format("Tag of interest is in sight!\n\nID: %d", tagOfInterest.id));
                    tagToTelemetry(tagOfInterest);
                    return tagOfInterest.id;
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    telemetry.addLine("(The tag has never been seen)");
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");
            }

            telemetry.update();
        }
        return 0;
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

