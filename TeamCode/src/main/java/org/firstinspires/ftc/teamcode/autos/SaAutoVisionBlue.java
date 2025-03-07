package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Vision Auto Blue", group = "Comp", preselectTeleOp = "New Tele-op Blue")
public class SaAutoVisionBlue extends OpMode {
    VisionSampleAuto sampleAuto;
    @Override
    public void init() {
        sampleAuto = new VisionSampleAuto(hardwareMap, telemetry, RobotSideEnum.Blue);
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
