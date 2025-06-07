package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Red", group = "Not Comp mode")
public class MainTeleopRed extends OpMode {
    Teleop teleop;
    @Override
    public void init() {
        teleop = new Teleop(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Red);
        teleop.init();
    }

    @Override
    public void loop() {
        teleop.loop();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
