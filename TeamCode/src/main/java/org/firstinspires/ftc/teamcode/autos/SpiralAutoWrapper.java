package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Spiral Auto", group = "  Spiral")
public class SpiralAutoWrapper extends OpMode {
    SpiralAuto spiralAuto;
    @Override
    public void init() {
        spiralAuto = new SpiralAuto(hardwareMap, telemetry, RobotSideEnum.Red);
        spiralAuto.init();
    }

    @Override
    public void init_loop() {
        spiralAuto.init_loop();
    }

    @Override
    public void loop() {
        spiralAuto.loop();
    }

    @Override
    public void start() {
        spiralAuto.start();
    }
}
