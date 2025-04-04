package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Disabled
@Autonomous(name = "Red 6 Specimen Auto Intake", group = " C2Comp", preselectTeleOp = "New Tele-op Blue")
public class Spec6IntakeAutoRed extends OpMode {
    SpecimenAuto6SpecIntake specimenAuto;
    @Override
    public void init() {
        specimenAuto = new SpecimenAuto6SpecIntake(hardwareMap, telemetry, RobotSideEnum.Red);
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
