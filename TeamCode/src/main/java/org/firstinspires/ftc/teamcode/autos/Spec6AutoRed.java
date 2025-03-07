package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Red 6 Specimen Auto", group = " R2Comp", preselectTeleOp = "New Tele-op Red")
public class Spec6AutoRed extends OpMode {
    SpecimenAuto6Spec specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAuto6Spec(hardwareMap, telemetry, RobotSideEnum.Red);
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
}
