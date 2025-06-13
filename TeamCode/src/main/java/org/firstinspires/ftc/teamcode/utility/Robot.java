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
        if (doTransfer) {
            switch (transferState) {
                case -1:
                    transferTimer.resetTimer();
                    transferState = 0;
                    break;
                case 0:
                    if (transferTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000.0) {
                        intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setTransferState(1);
                    }
                    break;
                case 1:
                    if ((intakeSystem.intakeUntil() && transferTimer.getElapsedTimeSeconds() > 0.1) || transferTimer.getElapsedTimeSeconds() > 0.5) {
                        intakeSystem.setIntakePower(0);

                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (transferTimer.getElapsedTimeSeconds() > .5) {
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                        setTransferState(4);
                    }
                    break;
                case 4:
                    if ((outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer()) || transferTimer.getElapsedTimeSeconds() > 0.75) {
                        setTransferState(5);
                    }
                    break;
                case 5:
                    if (transferTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeSystem.setIntakePower(0);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);

                        setTransferState(6);
                    }
                    break;
                case 6:
                    if (transferTimer.getElapsedTimeSeconds() > .3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);

                        setTransferState(7);
                    }
                    break;
                case 7:
                    if (outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar) {
                        atStorePreset = false;
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);

                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        setTransferState(8);
                    }
                    break;
                case 8:
                    if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 200) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArm);

                        doTransfer = false;
                        setTransferState(-1);
                    }
                    break;
            }
        }

        if (doGoWallFromStore) {
            switch (presetState) {
                case -1:
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    outtakeSystem.setArmPos(Constants.Outtake.downArm);
                    setPresetState(0);
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 100) {
                        outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        intakeSystem.setHSlidePos(0);
                        setPresetState(1);
                    }
                    break;
                case 1:
                    if (presetTimer.getElapsedTimeSeconds() > 0.1) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallAutoSlides);
                        setPresetState(2);
                    }
                    break;
                case 2:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.intakeWallAutoSlides) < 30) {
                        atStorePreset = false;
                        setPresetState(-1);
                    }
                    break;
            }
        }

        if (doIntakeWhilePark) {
            switch (parkState) {
                case 0:
                    if (intakeSystem.intakeUntil()) {
                        intakeSystem.storePos();
                        intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        doDriveShake = false;
                        parkState = 1;
                        parkTimer.resetTimer();
                    }
                    break;
                case 1:
                    if (parkTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000.0) {
                        intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        parkState = 2;
                        parkTimer.resetTimer();
                    }
                    break;
                case 2:
                    if (intakeSystem.intakeUntil()) {
                        intakeSystem.setIntakePower(0);
                        doDriveShake = false;
                        parkState = 3;
                        parkTimer.resetTimer();
                    }
                    break;
            }
        }


        if (doDriveShake) {
            if (shakeTimer.getElapsedTimeSeconds() > 1.1) {
                if (Math.round((shakeTimer.getElapsedTimeSeconds() - 1.1) * 2.3) % 2 == 0 ){
                    follower.setTeleOpMovementVectors(0,0, 1);
                } else {
                    follower.setTeleOpMovementVectors(0,0, -1);
                }
            }
        } else {
            shakeTimer.resetTimer();
        }
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
