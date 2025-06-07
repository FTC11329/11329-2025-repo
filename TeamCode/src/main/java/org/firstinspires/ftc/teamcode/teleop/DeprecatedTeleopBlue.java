package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Deprecated
@Disabled
@TeleOp(name = "Deprecated Tele-op Blue", group = "zComp mode")
public class DeprecatedTeleopBlue extends OpMode {
    DeprecatedTeleop deprecatedTeleop;
    @Override
    public void init() {
        deprecatedTeleop = new DeprecatedTeleop(hardwareMap, telemetry, gamepad1, gamepad2, RobotSideEnum.Blue);
        deprecatedTeleop.init();
    }

    @Override
    public void loop() {
        deprecatedTeleop.loop();
    }

    @Override
    public void start() {
        deprecatedTeleop.start();
    }

    @Override
    public void stop() {
        deprecatedTeleop.stop();
    }
}
