package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;

/**
 * This class will hold everything needed to run any part of the robot
 */
public class Robot {
    public Climber climber;
    public Follower follower;
    public Telemetry telemetry;
    public Attempt89 blockVision;
    public Drivetrain driveTrain;
    public PowerTakeOff powerTakeOff;
    public IntakeSystem intakeSystem;
    public OuttakeSystem outtakeSystem;

    public Robot(Climber climber, Follower follower, Telemetry telemetry, Attempt89 blockVision, Drivetrain driveTrain, PowerTakeOff powerTakeOff, IntakeSystem intakeSystem, OuttakeSystem outtakeSystem) {
        this.climber       = climber;
        this.follower      = follower;
        this.telemetry     = telemetry;
        this.driveTrain    = driveTrain;
        this.blockVision   = blockVision;
        this.powerTakeOff  = powerTakeOff;
        this.intakeSystem  = intakeSystem;
        this.outtakeSystem = outtakeSystem;
    }
}
