package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op Blue Replay Time", group = "   Comp mode")
public class TeleopBlueReplayTime extends OpMode {
    AutoReplayTeleopTimeBased teleop;
    @Override
    public void init() {
        teleop = new AutoReplayTeleopTimeBased(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Blue);
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
