package org.firstinspires.ftc.teamcode.Auton;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Autonomous Right Stack", group="Robot")

public class AutonRightStack extends AutonDriving{
    public double LIFTMOTORPOWER = 1.0;
    public AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {
        robot.init();
        robot.initGyro();
        runtime.reset();
        robot.flipToPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        tagOfInterest = getTag(robot.initAprilTagDetection());
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            pathWithoutCamera();
        }
        else
        {
            basicPathWithCamera(tagOfInterest);
            //todo: you need to make an actual path with camera that goes does the cone stuff then comes back.

        }

        sleep(50);
    }

    public void basicPathWithCamera(AprilTagDetection tagOfInterest){
        if(tagOfInterest.id == 1){
            encoderDrive(0.5, 270, 30,2);
            encoderDrive(0.5, 0,24,3);
        }
        else if(tagOfInterest.id == 2){
            encoderDrive(0.5, 270,
                    30,2);
        }
        else if(tagOfInterest.id == 3){
            encoderDrive(0.5, 270, 30,2);
            encoderDrive(0.5, 180,24,3);
        }
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
}

