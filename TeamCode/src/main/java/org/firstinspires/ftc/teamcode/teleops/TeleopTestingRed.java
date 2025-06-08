package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op test Red", group = "Comp mode")
public class TeleopTestingRed extends OpMode {
    TeleopTesting teleop;
    @Override
    public void init() {
        teleop = new TeleopTesting(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Red);
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
