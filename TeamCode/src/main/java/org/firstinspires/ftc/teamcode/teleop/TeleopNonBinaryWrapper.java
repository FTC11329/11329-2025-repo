package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Non Binary", group = " Comp mode")
public class TeleopNonBinaryWrapper extends OpMode {
    TeleopNonBinary teleop;
    @Override
    public void init() {
        teleop = new TeleopNonBinary(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Auto);
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
