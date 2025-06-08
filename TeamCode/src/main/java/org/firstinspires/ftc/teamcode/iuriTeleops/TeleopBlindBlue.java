package org.firstinspires.ftc.teamcode.iuriTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Blind Blue", group = "zComp mode")
public class TeleopBlindBlue extends OpMode {
    TeleopBlind teleop;
    @Override
    public void init() {
        teleop = new TeleopBlind(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Blue);
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
