package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Sample Vision Auto Red", group = " R3Comp", preselectTeleOp = "New Tele-op Red")
public class SaVisionAutoRed extends OpMode {
    SampleVisionAuto sampleAuto;
    @Override
    public void init() {
        sampleAuto = new SampleVisionAuto(hardwareMap, telemetry, RobotSideEnum.Red);
        sampleAuto.init();
    }

    @Override
    public void init_loop() {
        sampleAuto.init_loop();
    }

    @Override
    public void loop() {
        sampleAuto.loop();
    }

    @Override
    public void start() {
        sampleAuto.start();
    }
}
