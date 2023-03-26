package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonRunner.AutonomousDriving;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class DuoFieldCenter extends AutonomousDriving {
    public int currentPreset = 0;
    private static final double LIFTMOTORPOWER = 0.8;
    public ElapsedTime runtime = new ElapsedTime();
    public static double FLIPDELAY = 700; // milliseconds

    @Override
    public void runOpMode() {
        double clawPosition = 1;
        double flipPosition = 0;
        double time = 0;
        double rotatePosition = 0;
        boolean isRotated = true;
        boolean isFlipped = true;
        double speedMultiplier;
        boolean lock = false;
        boolean needToResetLift = false;

        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this, true);
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        robot.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        int offsetCounts = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Pose2d currentPose = robot.getPoseEstimate();
            // lock moving controls
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                lock = !lock;
            }

            // Drive robot via mecanum
            // Slow mode mapped to left bumper
            speedMultiplier = gamepad1.left_trigger > 0.3 ? 0.3 : 1;

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-currentPose.getHeading());
            robot.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if(lock)
                robot.setWeightedDrivePower(
                        new Pose2d(0, 0)
                );

            // Speed Multiplier Telemetry
            telemetry.addData("Speed Multiplier: ", speedMultiplier);

            // Claw mapped to a
            if (gamepad2.a && lift.upMotorL.getCurrentPosition() + lift.upMotorR.getCurrentPosition() > Lift.minHeightForFlip * 2) {
                lift.downDrop(Lift.heightsCounts[currentPreset] + offsetCounts);
                sleep(300);
                while(gamepad2.a) {}
            }
            else if(gamepad2.a) {
                clawPosition = 1 - clawPosition;
                while(gamepad2.a) {}
            }
            lift.claw.setPosition(clawPosition);

            // Flipping mapped to y
            if (gamepad2.y) { // flipPosition = 0 means default state
                flipPosition = 1 - flipPosition;
                while(gamepad2.y) {}
                isFlipped = false;
            }

            if ((lift.sensorPress()) && needToResetLift) {
                lift.stopAndResetLiftEncoders();
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                needToResetLift = false;
            }
            else if(!lift.sensorPress()){
                needToResetLift = true;
            }

            if(isFlipped){
                lift.flipToPosition(1 - flipPosition);
                lift.flipToPosition(flipPosition);
            }
            else if(lift.upMotorL.getCurrentPosition() + lift.upMotorR.getCurrentPosition() <= 2 * Lift.minHeightForFlip){
                lift.flipToPosition(flipPosition);
                lift.flipToPosition(1 - flipPosition);
            }
            else if(lift.upMotorL.getCurrentPosition() + lift.upMotorR.getCurrentPosition() > 2 * Lift.minHeightForFlip){
                lift.claw.setPosition(1);
                lift.flipToPosition(1 - flipPosition);
                lift.flipToPosition(flipPosition);
                time = runtime.milliseconds();
                isFlipped = true;
                isRotated = false;
            }

            //auto rotate after flip
            if(!isRotated && Math.abs(runtime.milliseconds() - time - FLIPDELAY) <= 175) {
                rotatePosition = 1 - rotatePosition;
                isRotated = true;
            }

            if(gamepad2.x){
                rotatePosition = 1 - rotatePosition;
                while(gamepad2.x){}
            }
            lift.setRotate(rotatePosition);

            // flip is asked for
            // set new flip position
            // queue a flip to occur
            // hold until above height
            // flip
            // start delay, queue rotate
            // hold until delay
            // rotate






            // Lifts
            telemetry.addData("leftLiftPos", lift.upMotorL.getCurrentPosition());
            telemetry.addData("rightLiftPos", lift.upMotorR.getCurrentPosition());

            // Offset Calculations using bumpers
            if(gamepad2.left_trigger > 0.3)
                offsetCounts -= 42;

            else if(gamepad2.right_trigger > 0.3)
                offsetCounts += 42;


            // dpad setting presets
            if(!gamepad2.dpad_up && dpadUpPressed){
                offsetCounts = 0;
                currentPreset = 1;
            }
            else if(!gamepad2.dpad_down && dpadDownPressed){
                offsetCounts = 0;
                currentPreset = 0;
            }
            else if(!gamepad2.dpad_right && dpadRightPressed){ // right is medium
                offsetCounts = 0;
                currentPreset = 3;
            }
            else if(!gamepad2.dpad_left && dpadLeftPressed){ // left is low
                offsetCounts = 0;
                currentPreset = 2;
            }
            dpadUpPressed = gamepad2.dpad_up;
            dpadDownPressed = gamepad2.dpad_down;
            dpadLeftPressed = gamepad2.dpad_left;
            dpadRightPressed = gamepad2.dpad_right;

            // Move Lifts
            telemetry.addData("Preset Mode", true);
            telemetry.addData("Current Preset", Lift.heightNames[currentPreset]);
            lift.setLift(Lift.heightsCounts[currentPreset] + offsetCounts, LIFTMOTORPOWER);

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(25);
        }
    }
}
