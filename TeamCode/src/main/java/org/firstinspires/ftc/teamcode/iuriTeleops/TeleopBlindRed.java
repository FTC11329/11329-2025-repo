package org.firstinspires.ftc.teamcode.iuriTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Blind Red", group = "zComp mode")
public class TeleopBlindRed extends OpMode {
    TeleopBlind teleop;
    @Override
    public void init() {
        teleop = new TeleopBlind(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Red);
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
