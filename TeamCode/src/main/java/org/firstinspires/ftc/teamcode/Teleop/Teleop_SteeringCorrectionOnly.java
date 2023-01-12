package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp
public class Teleop_SteeringCorrectionOnly extends OpMode {
    enum Drive_Modes {
        ECO_MODE,
        SPORT_MODE,
        N_MODE
    }

    enum Gyro_Modes {
        DRIVE_STRAIGHT,
        TURN
    }

    F0Hardware r = new F0Hardware();
    PID angleCorrector;

    Drive_Modes dm = Drive_Modes.ECO_MODE;
    Gyro_Modes gm = Gyro_Modes.TURN;

    boolean powerCut = false;

    ElapsedTime e;
    ElapsedTime powerCutTimer;
    double[] prevAngles;

    ElapsedTime correctionDelay = new ElapsedTime();
    ElapsedTime accelRamp = new ElapsedTime();

    @Override
    public void init() {
        e = new ElapsedTime();
        powerCutTimer = new ElapsedTime();
        r.init(hardwareMap);
        telemetry.addLine("imu reset in progress...");
        telemetry.update();
        r.imu.reset();
        telemetry.addLine("imu reset finished ^_^");
        telemetry.update();
        angleCorrector = new PID(new double[]{0.025,0,0.1}, r.imu.getHeading());
        prevAngles = r.imu.getAngles();
        telemetry.addLine("completely finished with init ^o^");
        telemetry.update();
    }
    @Override
    public void loop() {
        double[] angles = r.imu.getAngles();

        double accelInput = gamepad1.right_trigger - 0.4*gamepad1.left_trigger;
        double steerInput = gamepad1.left_stick_x;

        if(accelInput == 0.0)
        {
            accelRamp.reset();
        }

        switch(dm) {
            case ECO_MODE:
                accelInput *= 0.25;
                telemetry.addLine("Eco Mode");
                break;
            case SPORT_MODE:
                accelInput *= 0.5;
                telemetry.addLine("Sport Mode");
                break;
            case N_MODE:
                telemetry.addLine("N Mode");
                break;
        }

        // Drive mode changing
        if(gamepad1.b) {
            dm = Drive_Modes.ECO_MODE;
        } else if(gamepad1.x) {
            dm = Drive_Modes.SPORT_MODE;
        } else if(gamepad1.y) {
            dm = Drive_Modes.N_MODE;
        }

        // Handbrake
        if(gamepad1.a) {
            accelInput = -1*Math.signum(accelInput);
        }

        double lp, rp;
        double yawCorrector = angleCorrector.calculate(angles[0]);
        double steerCorrect = 0.0f;//used to slightly adjust where the robot corrects to on steer *EXPERIMENTAL*
        double steerCorrectInc = 0.01f;

        if(gamepad1.dpad_right)
        {
            steerCorrect += steerCorrectInc;
        }
        else if(gamepad1.dpad_left)
        {
            steerCorrect -= steerCorrectInc;
        }

        // Gyro angle correction
        if (gm == Gyro_Modes.DRIVE_STRAIGHT) {
            lp = accelInput - (accelInput/(accelInput-accelRamp.milliseconds()));
            rp = accelInput - (accelInput/(accelInput-accelRamp.milliseconds()));
            telemetry.addData("accelRamp", accelRamp.milliseconds());
            if(Math.abs(steerInput) > 0.03) {
                gm = Gyro_Modes.TURN;
            }
            if(accelInput != 0)
            {
                r.steer(yawCorrector*.75);
            }
        } else {
            angleCorrector.setRefVal(angles[0]);
            lp = accelInput*(1+0.3*steerInput);
            rp = accelInput*(1-0.3*steerInput);
            if(Math.abs(steerInput) < 0.03) {
                if(correctionDelay.milliseconds() > 250) {
                    gm = Gyro_Modes.DRIVE_STRAIGHT;
                }
            } else {
                correctionDelay.reset();
            }
            r.steer(steerInput*(1-0.3*accelInput)+steerCorrect);
        }

        r.left.setPower(lp);
        r.right.setPower(rp);


        double[] dA = new double[3];
        for(int i = 0; i < 3; i++) {
            dA[i] = angles[i] - prevAngles[i];
        }

        telemetry.addData("Angles:", Arrays.toString(angles));
        telemetry.addData("dA: ",Arrays.toString(dA));
        telemetry.addData("yawCorrector:",yawCorrector);
        telemetry.addData("Steer Correct", steerCorrect);
        telemetry.addLine(String.valueOf(powerCut));

        telemetry.update();
    }
    @Override
    public void stop() {
        r.stop();
    }
}
