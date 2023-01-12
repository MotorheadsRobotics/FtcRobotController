package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp
public class Teleop extends OpMode {
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
    boolean first = false;
    double[] prevAngles;

    ElapsedTime correctionDelay = new ElapsedTime();

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
        angleCorrector = new PID(new double[]{0.025,0,0.2}, r.imu.getHeading());
        prevAngles = r.imu.getAngles();
        telemetry.addLine("completely finished with init ^o^");
        telemetry.update();
    }
    @Override
    public void loop() {
        double[] angles = r.imu.getAngles();

        double accelInput = gamepad1.right_trigger - 0.6*gamepad1.left_trigger;
        double steerInput = gamepad1.left_stick_x;

        switch(dm) {
            case ECO_MODE:
                accelInput *= 0.25;
                telemetry.addLine("Eco Mode");
                break;
            case SPORT_MODE:
                accelInput *= 0.6;
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

        double angPowerAdder = 0, lp, rp;

        // Gyro angle correction
        if (gm == Gyro_Modes.DRIVE_STRAIGHT) {
            angPowerAdder = angleCorrector.calculate(angles[0]);
            lp = accelInput;
            rp = accelInput;
            if(accelInput > 0.05) {
                lp += angPowerAdder;
                rp -= angPowerAdder;
            }
            if(Math.abs(steerInput) > 0.03) {
                gm = Gyro_Modes.TURN;
            }
        } else {
            angleCorrector.setRefVal(angles[0]);
            lp = accelInput*(1-0.7*steerInput);
            rp = accelInput*(1-0.7*steerInput);

        }



        // Roll-protection
        /*
        if(first || powerCutTimer.milliseconds() < 30) {
            powerCut = true;
            lp = 0;
            rp = 0;
            prevAngles = angles;
            e.reset();
        } else {
            powerCut = false;
            double dt = e.milliseconds();
            e.reset();
            if(Math.abs(angles[1] - prevAngles[1])/dt > 0.01 || Math.abs(angles[2] - prevAngles[2])/dt > 0.01) {
                powerCutTimer.reset();
            }
        }

         */

        r.left.setPower(lp);
        r.right.setPower(rp);

        // Steering
        r.steer(steerInput*(1-0.7*accelInput));

        double[] dA = new double[3];
        for(int i = 0; i < 3; i++) {
            dA[i] = angles[i] - prevAngles[i];
        }

        telemetry.addData("Angles:", Arrays.toString(angles));
        telemetry.addData("dA: ",Arrays.toString(dA));
        telemetry.addData("angPowerAdder:",angPowerAdder);
        telemetry.addLine(String.valueOf(powerCut));

        telemetry.update();
    }
    @Override
    public void stop() {
        r.stop();
    }
}
