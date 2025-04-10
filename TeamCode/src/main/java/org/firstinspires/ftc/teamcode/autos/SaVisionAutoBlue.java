package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Disabled
@Autonomous(name = "Sample Vision Auto Blue", group = " B3Comp", preselectTeleOp = "New Tele-op Blue")
public class SaVisionAutoBlue extends OpMode {
    SampleVisionAuto sampleAuto;
    @Override
    public void init() {
        sampleAuto = new SampleVisionAuto(hardwareMap, telemetry, RobotSideEnum.Blue);
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
