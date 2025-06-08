package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Climber Reset", group = " Testing")
public class ClimberReset extends OpMode {

    FtcDashboard dashboard;
    Climber climber;
    Drivetrain driveTrain;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    PowerTakeOff powerTakeOff;

    boolean ptoToggle = false;
    boolean ptoDebounce = false;

    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap, RobotSideEnum.Blue, true);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Blue);
        powerTakeOff = new PowerTakeOff(hardwareMap);

        outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);

//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        intakeSystem.storePos();
    }

    @Override
    public void loop() {
        driveTrain.drive(-gamepad1.left_stick_y, 0, 0, DriveSpeedEnum.Fast);
        climber.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        outtakeSystem.manualVSlide(-gamepad1.right_stick_y);
        if (gamepad1.a) {
            outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
        }
        if (gamepad1.y) {
            outtakeSystem.setArmPos(Constants.Outtake.upArm);
        }

        if (gamepad1.back && !ptoDebounce) {
            ptoToggle = !ptoToggle;
            ptoDebounce = true;
        }
        if (!gamepad1.back) {
            ptoDebounce = false;
        }
        if (ptoToggle) {
            powerTakeOff.enable();
        } else {
            powerTakeOff.disable();
        }
        telemetry.addData("fl", driveTrain.getDriveCurrent()[0]);
        telemetry.addData("bl", driveTrain.getDriveCurrent()[1]);
        telemetry.addData("fr", driveTrain.getDriveCurrent()[2]);
        telemetry.addData("br", driveTrain.getDriveCurrent()[3]);

        telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
        telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDrivePowers()[0], driveTrain.getDrivePowers()[1]), Math.max(driveTrain.getDrivePowers()[2], driveTrain.getDrivePowers()[3])));
        telemetry.addData("Distance", climber.getDistance());
        telemetry.addData("Climb Pos", climber.getPos());
    }
}
