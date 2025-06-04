package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "New Tele-op Fast", group = " Comp mode")
public class NewTeleopFastWrapper extends OpMode {
    NewTeleopFAST teleop;
    @Override
    public void init() {
        teleop = new NewTeleopFAST(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Blue);
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
