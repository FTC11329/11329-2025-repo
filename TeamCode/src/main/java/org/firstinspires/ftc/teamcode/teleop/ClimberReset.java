package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;

@TeleOp(name = "Climber Reset", group = "Comp mode")
public class ClimberReset extends OpMode {

    Climber climber;
    Drivetrain driveTrain;
    OuttakeSystem outtakeSystem;
    PowerTakeOff powerTakeOff;

    boolean ptoToggle = false;
    boolean ptoDebounce = false;

    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
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
        telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDrivePowers()[0], driveTrain.getDrivePowers()[1]), Math.max(driveTrain.getDrivePowers()[2], driveTrain.getDrivePowers()[3])));
        telemetry.addData("Climb Pos", climber.getPos());
    }
}
