package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;

/**
 * This class will hold everything needed to run most parts of the robot for auto
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

    public Timer opmodeTimer;

    public int transferState = -1;
    public Timer transferTimer = new Timer();
    public boolean doTransfer = false;

    public int lowBarState = -1;
    public Timer lowBarTimer = new Timer();
    public boolean doLowBarFromWall = false;

    public int wallState = -1;
    public Timer wallTimer = new Timer();
    public boolean doWallFromLowBar = false;

    public int wallState = -1;
    public Timer wallTimer = new Timer();
    public boolean doWallFromLowBar = false;

    public int presetState = -1;
    public Timer presetTimer = new Timer();
    public boolean doGoWallFromStore = false;

    public int parkState = 0;
    public Timer parkTimer = new Timer();
    public boolean doIntakeWhilePark = false;

    public Timer shakeTimer = new Timer();
    public boolean doDriveShake = false;
    public boolean atStorePreset = false;

    public Robot(Climber climber, Follower follower, Telemetry telemetry, Attempt89 blockVision, Drivetrain driveTrain, PowerTakeOff powerTakeOff, IntakeSystem intakeSystem, OuttakeSystem outtakeSystem) {
        this.climber       = climber;
        this.follower      = follower;
        this.telemetry     = telemetry;
        this.driveTrain    = driveTrain;
        this.blockVision   = blockVision;
        this.powerTakeOff  = powerTakeOff;
        this.intakeSystem  = intakeSystem;
        this.outtakeSystem = outtakeSystem;

        opmodeTimer = new Timer();
    }

    public void loop() {


        //Only for 
    }

    private void setTransferState(int set) {
        transferTimer.resetTimer();
        transferState = set;
    }
    private void setPresetState(int set) {
        presetTimer.resetTimer();
        presetState = set;
    }

    public void start() {
        opmodeTimer.resetTimer();
    }

}
