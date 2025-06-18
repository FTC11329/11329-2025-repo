package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Disabled
@Autonomous(name = "Blue 6 SUPER Specimen Auto", group = " B2Comp", preselectTeleOp = "New Tele-op Blue")
public class Spec6SuperAutoBlue extends OpMode {
    SpecimenAutoSuper6Spec specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAutoSuper6Spec(hardwareMap, telemetry, RobotSideEnum.Blue);
        specimenAuto.init();
    }

    @Override
    public void init_loop() {
        specimenAuto.init_loop();
    }

    @Override
    public void loop() {
        specimenAuto.loop();
    }

    @Override
    public void start() {
        specimenAuto.start();
    }

    @Override
    public void stop() {
        specimenAuto.stop();
    }
}
