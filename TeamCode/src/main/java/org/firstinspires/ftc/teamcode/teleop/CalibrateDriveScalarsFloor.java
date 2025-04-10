package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CalibrateDriveScalarsFloor", group = "Calibration")
public class CalibrateDriveScalarsFloor extends LinearOpMode {

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

        double testPower = 1; // set to 1 because that is what we drive at
        long runTimeMs = 3000;

        double[] distances = new double[4];

        for (int i = 0; i < 4; i++) motors[i].setPower(testPower);

        sleep(runTimeMs);

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
            distances[i] = Math.abs(motors[i].getCurrentPosition());
        }

        // Step 1: Find the max distance (fastest motor)
        double maxDist = 0;
        for (double d : distances) {
            if (d > maxDist) maxDist = d;
        }

        // Step 2: Compute ratio to the max (will all be ≤ 1)
        double[] percentSpeed = new double[4];
        for (int i = 0; i < 4; i++) {
            percentSpeed[i] = distances[i] / maxDist;
        }

        // Step 3: Find slowest motor
        double minSpeed = Double.MAX_VALUE;
        for (double n : percentSpeed) {
            if (n < minSpeed) minSpeed = n;
        }

        //Step 4: Convert speed ratio to scalar multiplier to make all motors run at speed of the slowest motor
        double[] scalars = new double[4];
        for (int i = 0; i < 4; i++) {
            scalars[i] = minSpeed / percentSpeed[i];  // 0 < scalar ≤ 1
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
