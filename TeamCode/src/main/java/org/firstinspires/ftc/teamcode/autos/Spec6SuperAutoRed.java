package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Disabled
@Autonomous(name = "Red 6 SUPER Specimen Auto", group = " R2Comp", preselectTeleOp = "New Tele-op Red")
public class Spec6SuperAutoRed extends OpMode {
    SpecimenAutoSuper6Spec specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAutoSuper6Spec(hardwareMap, telemetry, RobotSideEnum.Red);
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
