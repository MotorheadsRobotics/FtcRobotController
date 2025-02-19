package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Chassis;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Left Stack Regular", group="Robot")

public class AutonLeftStack extends AutonDriving{
    Lift lift;
    public double LIFTMOTORPOWER = 1.0;
    public AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {
        robot = new Chassis(this, true);
        lift = new Lift(this, true);
        Camera camera = new Camera(this);
        runtime.reset();
        sleep(250);
        lift.flipToPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        tagOfInterest = getTag(camera.initAprilTagDetection());
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
            telemetry.addData("Path Complete", "Yay! ");
            telemetry.update();
            sleep(10000);
        }
        else
        {
            pathWithCamera(tagOfInterest);
            telemetry.addData("Path Complete", "Yay! ");
            telemetry.update();
            sleep(10000);
        }

        sleep(50);
    }

    public void pathWithCamera(AprilTagDetection tagOfInterest){
        lift.closeClaw();
        moveConeToHighTerminalSimple();
        encoderDrive(0.5, 0, 9,2);
        lift.setLift(5,0.8);
        encoderDrive(0.5,270,55,3);

        if(tagOfInterest.id == 1){
            encoderDrive(0.5, 270, 4, 1);
            encoderDrive(0.5, 0,34,3);
        }
        else if(tagOfInterest.id == 2){

        }
        else if(tagOfInterest.id == 3){
            encoderDrive(0.5, 270, 4, 1);
            encoderDrive(0.5, 180,34,3);
        }


    }

    public void squarePathSignal(AprilTagDetection tagOfInterest){
        if(tagOfInterest.id == 1){
            encoderDrive(0.5, 90, 54,2);
            encoderDrive(0.5, 270, 4, 2);
            encoderDrive(0.5, 0,34,3);
        }
        else if(tagOfInterest.id == 2){
            encoderDrive(0.5, 90,
                    54,2);
        }
        else if(tagOfInterest.id == 3){
            encoderDrive(0.5, 90, 54,2);
            encoderDrive(0.5, 270, 4, 2);
            encoderDrive(0.5, 180,34,3);
        }
    }

    public void pathWithoutCamera() {
        lift.closeClaw(); // close claw
//        moveConeToHighTerminal(false);
        moveConeToHighTerminalSimple();
//        for(int numTimes = 4; numTimes > 3; numTimes--) { // change to numTimes >= 0 to do all five cones
//            moveToStack(numTimes);
//            moveBackToHighTerminal();
//        }
        encoderDrive(0.5, 0, 13,5);
        encoderDrive(0.5,270,55,5);

        lift.setLift(5,0.8,5);
    }

    public void moveConeToHighTerminalSimple(){
        encoderDrive(0.5, 90, 109.5,5);
        encoderDrive(0.5, 270, 6,5);

        lift.setLift(Lift.maxHeightInch * Lift.liftCountsPerInch,LIFTMOTORPOWER, 3);
        lift.flipToPosition(0);

        encoderDrive(0.5, 180, 12.5,2);

        lift.setLift((Lift.highInch - 3) * Lift.liftCountsPerInch,LIFTMOTORPOWER,1); // added

        lift.openClaw(); // open claw
        sleep(150);

        lift.setLift(Lift.highInch * Lift.liftCountsPerInch,LIFTMOTORPOWER,1);

        sleep(250);
        telemetry.addData("Path Part 1: ", "Done");
        telemetry.update();
    }
    public void moveBackToHighTerminal() {
        lift.closeClaw();
        lift.setLift(2741,LIFTMOTORPOWER);
        boolean dontFlip = true;
        sleep(100);
        // TODO: This the movement back from stack to terminal, should be opposite of moveToStack encoder functions
        encoderDriveNoWaiting(0.3,180,48);
        boolean dontStop = true;
        lift.flipToPosition(1); // flip

        while(dontStop || dontFlip){
            dontStop = robot.fLMotor.isBusy() || robot.fRMotor.isBusy() || robot.bLMotor.isBusy() || robot.bRMotor.isBusy();
            dontFlip = lift.upMotorL.getCurrentPosition() + lift.upMotorR.getCurrentPosition() < 2 * Lift.minHeightForFlip;
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
                lift.setRotate(1); // rotate
                lift.openClaw(); // open
            }
        }
        // TODO: strafe back to next to terminal
        encoderDrive(0.3,90, 12,1);
        // TODO: Move until cone is on top of high terminal
        encoderDrive(0.5,0,6,1);
        lift.openClaw();

    }
    public void moveToStack(int numTimes) {
        // TODO: move out of the way into center of tile
        encoderDrive(0.3,270, 12,1);
        //move lift back down
        lift.setLift(numTimes * 440,LIFTMOTORPOWER); // stack cone #5
        lift.closeClaw();
        lift.setRotate(0);
        // TODO: begin moving toward stack, inches need to get robot to wall stack
        encoderDriveNoWaiting(0.3,0,48);
        //prepare claw to pick up cones
        sleep(250);
        lift.flipToPosition(0); // back to normal flipped state
        sleep(250);
        lift.openClaw();
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
    public void moveConeToHighTerminal(boolean isRightSide){
        lift.setLift(9855,LIFTMOTORPOWER);
        boolean dontFlip = true;
        lift.flipToPosition(1); // flip
        // TODO: initial movement, should get robot to right next to the high terminal
        //  (robot should be positioned with claw facing right, centered in the tile)
        encoderDriveNoWaiting(0.7,90,125);
        boolean dontStop = true;

        while(dontStop || dontFlip){
            dontStop = robot.motorsBusy();
            dontFlip = lift.upMotorL.getCurrentPosition() + lift.upMotorR.getCurrentPosition() < 2 * Lift.minHeightForFlip;
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
                lift.setRotate(1);
            }
        }
        encoderDrive(0.7,270,20,1);
        if(!isRightSide)
            turnDegrees(0.5,180,1);
        // TODO: Move until cone is on top of high terminal
        int direction = isRightSide ? 0 : 180;
        encoderDrive(0.5,direction,6,1);
        lift.openClaw();
        sleep(250);
    }
}

