package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Red", group = " Comp mode")
public class TeleopRed extends OpMode {
    Teleop teleop = new Teleop(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Red);
    @Override
    public void init() {
        teleop.init();
    }

    @Override
    public void loop() {
        teleop.loop();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
