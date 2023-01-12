package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SteeringTest extends LinearOpMode {
    Servo s;

    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(Servo.class, "steer");

        waitForStart();

        ElapsedTime e = new ElapsedTime();

       while(!isStopRequested()) {
           s.setPosition(0);
       }

        /*

        for(int i = 0; i < 50; i++) {
            e.reset();
            while(e.milliseconds() < 500 && !isStopRequested()) {
                double angle = i/50.0;
                s.setPosition(angle);
                telemetry.addData("angle",angle);
                telemetry.update();
            }
        }

         */
    }
}
