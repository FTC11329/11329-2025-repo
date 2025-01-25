package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;

@TeleOp(name = "Climber Reset", group = " Comp mode")
public class ClimberReset extends OpMode {

    Climber climber;
    Drivetrain drivetrain;
    PowerTakeOff powerTakeOff;

    boolean ptoToggle = false;
    boolean ptoDebounce = false;

    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(-gamepad1.left_stick_y, 0, 0, DriveSpeedEnum.Fast);
        climber.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

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

        if (gamepad1.y) {
            powerTakeOff.hookRelease();
        } else {
            powerTakeOff.hookGrab();
        }
    }
}
