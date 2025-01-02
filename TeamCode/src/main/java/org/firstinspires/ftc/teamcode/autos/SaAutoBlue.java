package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.Teleop;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Sample Auto Blue", group = " Comp", preselectTeleOp = "Tele-op Blue")
public class SaAutoBlue extends OpMode {
    SampleAuto sampleAuto;
    @Override
    public void init() {
        sampleAuto = new SampleAuto(hardwareMap, telemetry, RobotSideEnum.Blue);
        sampleAuto.init();
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
