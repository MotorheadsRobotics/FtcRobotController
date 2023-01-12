package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Old Drive Software")

public class Teleop_old extends OpMode {
    F0Hardware f = new F0Hardware();

    double[] maxP = new double[] {.1, .25, .5, .75, .9};
    int powerIndex = 0;
    String[] powerSetting = new String[] {"Eco Mode", "Normal Mode", "Sport Mode", "Track Mode", "Zero Mode"};

    boolean wasPressedLeft = true;
    boolean wasPressedRight = true;
    boolean wasPressedUp = true;
    boolean wasPressedDown = true;
    boolean powerChanged;

    double turnCorrK = 0.018; //turn correction magnitude: 0.01 - 0.04
    double gyroDelay = 100; //delay before veer correction (ms)
    double brakePower = -0.025;
    double accel;

    double turnTrim = 0;
    double lastHeading = 0;

    boolean gyroReset = false, ramping = false, gyroWait = false;

    ElapsedTime gyroRT = new ElapsedTime();

    ElapsedTime t = new ElapsedTime();

    @Override
    public void init() {
        f.init(hardwareMap);

        String status;
        if (f.imu.getRevIMU().isGyroCalibrated())
            status = "Calibrated";
        else
            status = "Calibration Error - Check Gyroscope";

        telemetry.addData("F0 Drive Console Version 2.3.1 Status", status);
        telemetry.addData("Credits", "Timmy Nadolsky and Christopher Dycus");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Power Settings
        if (!(gamepad1.dpad_left == wasPressedLeft) && gamepad1.dpad_left && maxP[powerIndex] > 0.05) {
            powerIndex--;
        } else if (!(gamepad1.dpad_right == wasPressedRight) && gamepad1.dpad_right && maxP[powerIndex] < 1) {
            powerIndex++;
        }
        if ((powerIndex > 4)) {
            powerIndex = 4;
            powerChanged = true;
        } else if (powerIndex < 0) {
            powerIndex = 0;
            powerChanged = true;
        }

        if(powerChanged) {
            //correction adjustment based on power setting
            switch (powerIndex) {
                case 0:
                    turnCorrK = 0.03;
                    break;
                case 1:
                    turnCorrK = 0.025;
                    break;
                case 2:
                    turnCorrK = 0.0225;
                    break;
                case 3:
                    turnCorrK = 0.02;
                    break;
                default:
                    turnCorrK = 0.018;
                    break;
            }
            powerChanged = false;
        }

        telemetry.addData("Power Setting", powerSetting[powerIndex]);

        wasPressedLeft = gamepad1.dpad_left;
        wasPressedRight = gamepad1.dpad_right;

        //turn correction adjustment control
        if (!(gamepad1.dpad_up == wasPressedUp) && gamepad1.dpad_up)
            turnCorrK =+ 0.001;
        else if (!(gamepad1.dpad_down == wasPressedDown) && gamepad1.dpad_down)
            turnCorrK =- 0.001;
        if (turnCorrK < 0.01)
            turnCorrK = 0.01;
        else if (turnCorrK > 0.05)
            turnCorrK = 0.05;

        wasPressedUp = gamepad1.dpad_up;
        wasPressedDown = gamepad1.dpad_down;

        telemetry.addData("AutoSteer Percentage", Math.round((turnCorrK-0.01)*100/0.04));

        double accel = maxP[powerIndex] * (gamepad1.right_trigger - gamepad1.left_trigger);
        double turn = gamepad1.left_stick_x;

        if (turn != 0) {
            gyroReset = false;
        } else {
            if (!gyroReset) { //checking for steering input
                if(!gyroWait) {
                    gyroWait = true;
                    gyroRT.reset();
                }
                else if(gyroRT.milliseconds() > gyroDelay) { //gyro delay for correction after steering input
                    gyroWait = false;
                    lastHeading = f.imu.getHeading();
                    gyroReset = true;
                }
            } else {
                double corr = (f.imu.getHeading() - lastHeading) * turnCorrK;
                turn = MathUtils.clamp(turn + corr, -1, 1);
            }
        }

        double rp, lp;

        //defining right and left motor powers
        rp = accel * (1 - 0.7 * turn);
        lp = accel * (1 + 0.7 * turn);

        //motor power control
        if(gamepad1.b) {
            f.left.setPower(brakePower);
            f.left.setPower(brakePower);
            telemetry.addData("Throttle Status", "BRAKE ENGAGED");
        } else if (gamepad1.a && powerIndex == 5){
            accel = (gamepad1.right_trigger - gamepad1.left_trigger);
            rp = accel * (1 - 0.7 * turn);
            lp = accel * (1 + 0.7 * turn);
            f.left.setPower(MathUtils.clamp(rp,-1,1));
            f.right.setPower(MathUtils.clamp(lp,-1,1));
            telemetry.addData("Turbo Engaged", "Power Limiter Disabled");
        } else {
            f.left.setPower(MathUtils.clamp(rp,-1,1));
            f.right.setPower(MathUtils.clamp(lp,-1,1));
            telemetry.addData("Throttle Percentage", Math.round(lp*100));
        }

        //turn correction adjustment control
        if (!(gamepad1.dpad_up == wasPressedUp) && gamepad1.dpad_up)
            turnCorrK =+ 0.001;
        else if (!(gamepad1.dpad_down == wasPressedDown) && gamepad1.dpad_down)
            turnCorrK =- 0.001;
        if (turnCorrK < 0.01)
            turnCorrK = 0.01;
        else if (turnCorrK > 0.05)
            turnCorrK = 0.05;

        wasPressedUp = gamepad1.dpad_up;
        wasPressedDown = gamepad1.dpad_down;

        telemetry.addData("Steering Gyro-Assist Percentage", Math.round((turnCorrK-0.01)*100/0.04));

        //steering trim
        if(gamepad1.x) {
            turnTrim += 0.002;
        } else if(gamepad1.y) {
            turnTrim -= 0.002;
        }

        //Steering
        InterpLUT servoPosition = new InterpLUT();
        servoPosition.add(-1.01,-1);
        servoPosition.add(0,0);
        servoPosition.add(1.01, 1.5);
        servoPosition.createLUT();
        InterpLUT speedCorrection = new InterpLUT();
        speedCorrection.add(-0.01,1);
        speedCorrection.add(1.01, 0.2);
        speedCorrection.createLUT();
        // Steering
        f.steer(turn*(1-0.7*turn));

        telemetry.addData("Top Speed Reached (mph)", (f.imu.getRevIMU().getVelocity().xVeloc)*2.2);

        telemetry.update();
    }

    public void stop() {
        f.left.setPower(0);
        f.right.setPower(0);
    }
}
