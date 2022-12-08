package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Autonomous Right Stack", group="Robot")

public class AutonRightStack extends IterativeAuton {
    public double LIFTMOTORPOWER = 1.0;

    @Override
    public void start() {
        hasStarted = true;
        runtime.reset();
        telemetry.addData("Message: ", robot.message);
        telemetry.update();
        sleep(1000);
        finalMessage = robot.message;

        robot.claw.setPosition(1);

        encoderDrive(1,270,29, 2);
        if(finalMessage == null){
            finalMessage = robot.message;
        }
        if(finalMessage != null){
            switch (finalMessage) {
                case "https://left.com":
                    // strafe left one tile
                    encoderDrive(1, 180, 24, 1.5);
                    break;
                case "https://middle.com":
                    // no need to move
                    break;
                case "https://right.com":
                    // strafe right one tile
                    encoderDrive(1, 0, 24, 1.5);
                    break;
                default:
                    // hope it's middle, attempting to recheck
                    telemetry.addData("We got used by another team :(", "unfortunate L");
                    telemetry.update();
            }
        }
        // path done
        robot.setLift(10560, LIFTMOTORPOWER);
        robot.flipL.setPosition(0);
        robot.flipR.setPosition(1); // flip
        while(robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() < 2 * robot.minHeightForFlip) {}
        robot.rotate.setPosition(0); // rotate
        robot.claw.setPosition(0); // open

    }
}
