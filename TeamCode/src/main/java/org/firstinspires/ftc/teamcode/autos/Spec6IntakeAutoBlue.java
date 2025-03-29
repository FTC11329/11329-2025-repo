package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Blue 6 Specimen Auto Intake", group = " C2Comp", preselectTeleOp = "New Tele-op Blue")
public class Spec6IntakeAutoBlue extends OpMode {
    SpecimenAuto6SpecIntake specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAuto6SpecIntake(hardwareMap, telemetry, RobotSideEnum.Blue);
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
