package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "CalibrateDriveScalarsFloor", group = "zCalibration")
public class CalibrateDriveScalarsFloor extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotor[] motors = {leftFront, rightFront, leftBack, rightBack};

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Ready to calibrate. Press play.");
        telemetry.update();

        waitForStart();

        double testPower = 1.0;
        long runTimeMs = 3000;
        double runTimeSec = runTimeMs / 1000.0;

        double[] distances = new double[4];

        for (DcMotor motor : motors) motor.setPower(testPower);
        sleep(runTimeMs);
        for (DcMotor motor : motors) motor.setPower(0);

        for (int i = 0; i < 4; i++) {
            distances[i] = Math.abs(motors[i].getCurrentPosition());
        }

        // Step 1: Calculate velocities (ticks/sec)
        double[] velocities = new double[4];
        for (int i = 0; i < 4; i++) {
            velocities[i] = distances[i] / runTimeSec;
        }

        // Step 2: Find the slowest motor's velocity
        double minVelocity = velocities[0];
        for (double v : velocities) {
            if (v < minVelocity) minVelocity = v;
        }

        // Step 3: Create scalar to bring all motors to slowest motor's speed
        double[] scalars = new double[4];
        for (int i = 0; i < 4; i++) {
            scalars[i] = minVelocity / velocities[i];  // Always ≤ 1.0
        }

        // Output scalars to telemetry
        telemetry.addLine("Calibration complete:");
        telemetry.addData("Left Front Scalar", scalars[0]);
        telemetry.addData("Right Front Scalar", scalars[1]);
        telemetry.addData("Left Back Scalar", scalars[2]);
        telemetry.addData("Right Back Scalar", scalars[3]);
        telemetry.update();

        // Keep OpMode alive so you can read telemetry
        while (opModeIsActive()) {
            idle();
        }
    }
}
