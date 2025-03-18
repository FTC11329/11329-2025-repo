package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.Specimen5Sam1AutoEnum;

@Disabled
@Autonomous(name = "Blue 1 Sample 5 Specimen Auto", group = " B1Comp", preselectTeleOp = "New Tele-op Blue")
public class Spec5Sam1AutoBlue extends OpMode {
    SpecimenAuto5Spec1Sam specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAuto5Spec1Sam(hardwareMap, telemetry, RobotSideEnum.Blue);
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
