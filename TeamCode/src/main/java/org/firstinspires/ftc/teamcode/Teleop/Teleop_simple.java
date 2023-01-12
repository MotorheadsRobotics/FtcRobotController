package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop_simple extends OpMode {
    F0Hardware r = new F0Hardware();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        double a = gamepad1.right_trigger-0.3*gamepad1.left_trigger;
        double s = Math.signum(gamepad1.left_stick_x)*Math.pow(gamepad1.left_stick_x,2);

        r.left.setPower(a + Math.pow(a,3)*s);
        r.right.setPower(a - Math.pow(a,3)*s);

        // Steering
        r.steer(s*Math.abs(1-Math.pow(a,2)));


    }
}
