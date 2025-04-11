package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CalibrateDriveScalars", group = "zCalibration")
public class CalibrateDriveScalars extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotor[] motors = {leftFront, rightFront, leftBack, rightBack};

        // Reset and set to run using encoders
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();

        // Run each motor individually for a set time
        double testPower = 0.8;
        long runTimeMs = 5000;

        double[] distances = new double[4];

        for (int i = 0; i < 4; i++) {
            telemetry.addLine("Testing motor " + i);
            telemetry.update();

            motors[i].setPower(testPower);
            sleep(runTimeMs);
            motors[i].setPower(0);

            distances[i] = Math.abs(motors[i].getCurrentPosition());

            // Reset for the next motor
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(500);
        }

        // Step 1: Find the max distance (fastest motor)
        double maxDist = Double.MAX_VALUE;
        for (double d : distances) {
            if (d > maxDist) maxDist = d;
        }

        // Step 2: Compute scalars as ratios to the max (will all be ≤ 1)
        double[] scalars = new double[4];
        for (int i = 0; i < 4; i++) {
            scalars[i] = distances[i] / maxDist;  // 0 < scalar ≤ 1
        }


        telemetry.addLine("Calibration complete:");
        telemetry.addData("Left Front Scalar", scalars[0]);
        telemetry.addData("Right Front Scalar", scalars[1]);
        telemetry.addData("Left Back Scalar", scalars[2]);
        telemetry.addData("Right Back Scalar", scalars[3]);
        telemetry.update();

        // Wait to read values
        while (opModeIsActive()) {
            idle();
        }
    }
}
