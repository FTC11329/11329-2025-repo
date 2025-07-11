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
    public StateMachine stateMachine;
    public IntakeSystem intakeSystem;
    public OuttakeSystem outtakeSystem;
    public RobotStateVariables robotState;

    public Timer opmodeTimer;


    public Timer storeTimer = new Timer();
    public int storeState = -1;

    private int lowBarState = -1;
    private Timer lowBarTimer = new Timer();

    private Timer lowBasketTimer = new Timer();
    private int lowBasketState = -1;

    private Timer highBasketTimer = new Timer();
    private int highBasketState = -1;

    private Timer wallTimer = new Timer();
    private int wallState = -1;

    private Timer transferTimer = new Timer();
    private int transferState = -1;

    private Timer unStoringTimer = new Timer();
    private int unStoringState = -1;

    private int parkState = 0;
    private Timer parkTimer = new Timer();
    public boolean doIntakeWhilePark = false;

    private Timer shakeTimer = new Timer();
    public boolean doDriveShake = false;

    public boolean inAuto;

    public Robot(Climber climber, Telemetry telemetry, Drivetrain driveTrain, PowerTakeOff powerTakeOff, IntakeSystem intakeSystem, StateMachine stateMachine, OuttakeSystem outtakeSystem, RobotStateVariables robotState, boolean isAuto) {
        this(climber, null, telemetry, null, driveTrain, powerTakeOff, intakeSystem, stateMachine, outtakeSystem, robotState, isAuto);
    }

    public Robot(Climber climber, Follower follower, Telemetry telemetry, Attempt89 blockVision, Drivetrain driveTrain, PowerTakeOff powerTakeOff, IntakeSystem intakeSystem, StateMachine stateMachine, OuttakeSystem outtakeSystem, RobotStateVariables robotState, boolean isAuto) {
        this.climber       = climber;
        this.follower      = follower;
        this.telemetry     = telemetry;
        this.robotState    = robotState;
        this.driveTrain    = driveTrain;
        this.blockVision   = blockVision;
        this.powerTakeOff  = powerTakeOff;
        this.intakeSystem  = intakeSystem;
        this.stateMachine  = stateMachine;
        this.outtakeSystem = outtakeSystem;

        opmodeTimer = new Timer();

        inAuto = isAuto;
    }

    double timeToWait;
    public void loop() {
        if (stateMachine.doGoToStore()) {
            switch (storeState) {
                case -1:
                    timeToWait = -0.423 * (outtakeSystem.getArmPos() - 1.115);
                    timeToWait += 0.2;
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    robotState.clawToggle = true;
                    if (robotState.whereAmI == PlacePosEnum.highSpecimen) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                        outtakeSystem.setWristPos(Constants.Outtake.maxWrist);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    }
                    if ((!inAuto || intakeSystem.getHSlidePos() < Constants.Intake.transferSlides) && stateMachine.getBringSlidesIn()) {
                        intakeSystem.storeOutPos();
                    }
                    storeState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 40) {
                        outtakeSystem.setArmPos(Constants.Outtake.downArm);
                        storeTimer.resetTimer();
                        storeState = 1;
                    }
                    break;
                case 1:
                    if (storeTimer.getElapsedTimeSeconds() > timeToWait) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        outtakeSystem.setWristPos(Constants.Outtake.intakeWrist);
                        storeTimer.resetTimer();
                        storeState = 2;
                    }
                    break;
                case 2:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeAtIntakeSlides) < 40) {
                        if (stateMachine.bringSlidesIn) {
                            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        }
                        outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        robotState.clawToggle = false;
                        storeTimer.resetTimer();
                        storeState = 3;
                    }
                    break;
                case 3:
                    if (storeTimer.getElapsedTimeSeconds() > 0.1) {
                        robotState.whereAmI = PlacePosEnum.intake;
                        storeState = -1;
                        stateMachine.finishGoToStore();
                    }
                    break;
            }
        }

        if (stateMachine.doTransfer()) {
            switch (transferState) {
                case -1:
                    if (!intakeSystem.readyToTransfer()) {
                        transferTimer = new Timer();
                    }
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                    outtakeSystem.placePos(PlacePosEnum.intake);
                    robotState.clawToggle = false;
                    transferTimer.resetTimer();
                    transferState = 1;
                    break;
                case 1:
                    if (intakeSystem.readyToTransfer(false)) {
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                        transferTimer.resetTimer();
                        transferState = 2;
                    }
                    break;
                case 2:
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                    if (transferTimer.getElapsedTimeSeconds() > 0.1) {
                        // Continue
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        transferTimer.resetTimer();
                        transferState = 3;
                    }
                    if (!inAuto && transferTimer.getElapsedTimeSeconds() > 2.5) {
                        // Fail (not possible)
                        transferState = -1;
                        robotState.hasInIntake = false;
                        stateMachine.failTransfer();
                    }
                    if (inAuto && transferTimer.getElapsedTimeSeconds() > 0.75) {
                        // Auto continue (not possible)
                        transferTimer.resetTimer();
                        transferState = 3;
                    }
                    break;
                case 3:
                    if (transferTimer.getElapsedTimeSeconds() > 0.25) {
                        robotState.clawToggle = true;
                        intakeSystem.setIntakePower(0);
                        robotState.hasInIntake = false;
                        robotState.hasInOutake = true;
                        transferState = -1;
                        stateMachine.finishTransfer();

                    }
                    break;
            }
        }

        if (stateMachine.doUnStoreFromIntake()) {
            switch (unStoringState) {
                case -1:
                    if (stateMachine.goingHighBasket()) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    }
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                    outtakeSystem.setArmPos(Constants.Outtake.downArm);
                    outtakeSystem.setWristPos(Constants.Outtake.straightWrist);
                    unStoringTimer.resetTimer();
                    unStoringState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 60) {
                        outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        intakeSystem.setHSlidePos(0);
                        unStoringTimer.resetTimer();
                        unStoringState = 1;
                    }
                    break;
                case 1:
                    if (unStoringTimer.getElapsedTimeSeconds() > 0.3) {
                        robotState.whereAmI = PlacePosEnum.clear;
                        stateMachine.finishUnStoreFromIntake();
                        unStoringState = -1;
                    }
                    break;
            }
        }
        if (stateMachine.doUnStoreFromLowBar()) {
            switch (unStoringState) {
                case -1:
                    if (stateMachine.goingHighBasket()) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    }
                    outtakeSystem.setArmPos(Constants.Outtake.downArm);
                    outtakeSystem.setWristPos(Constants.Outtake.safeSpecimenWristLow);
                    unStoringTimer.resetTimer();
                    unStoringState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 40) {
                        outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                        outtakeSystem.setWristPos(Constants.Outtake.maxWrist);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        intakeSystem.setHSlidePos(0);
                        unStoringTimer.resetTimer();
                        unStoringState = 1;
                    }
                    break;
                case 1:
                    if (unStoringTimer.getElapsedTimeSeconds() > 0.3) {
                        robotState.whereAmI = PlacePosEnum.clear;
                        stateMachine.finishUnStoreFromLowBar();
                        unStoringState = -1;
                    }
                    break;
            }
        }

        if (stateMachine.doSafeHighSpecimen()) {
            outtakeSystem.placePos(PlacePosEnum.safeHighSpecimen);
            robotState.whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishSafeHighSpecimen();
        }

        if (stateMachine.doSafeLowSpecimen()) {
            outtakeSystem.placePos(PlacePosEnum.safeLowSpecimen);
            robotState.whereAmI = PlacePosEnum.safeLowSpecimen;
            stateMachine.finishSafeLowSpecimen();
        }

        if (stateMachine.doLowSpecimen()) {
            switch (lowBarState) {
                case -1:
                    if (robotState.whereAmI == PlacePosEnum.safeLowSpecimen) {
                        outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        break;
                    }
                    if (robotState.whereAmI != PlacePosEnum.wall) {
                        timeToWait = -0.423 * (outtakeSystem.getArmPos() - 1.115);
                        timeToWait += 0.4;
                    } else {
                        timeToWait = 0.8;
                    }
                    outtakeSystem.setWristPos(Constants.Outtake.maxWrist);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    robotState.clawToggle = true;
                    if (robotState.whereAmI == PlacePosEnum.highSpecimen) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    }
                    lowBarState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 40) {
                        outtakeSystem.setArmPos(Constants.Outtake.downArm);
                        lowBarTimer.resetTimer();
                        lowBarState = 1;
                    }
                    break;
                case 1:
                    if (lowBarTimer.getElapsedTimeSeconds() > timeToWait) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeAtIntakeSlides);
                        outtakeSystem.setWristPos(Constants.Outtake.straightWrist);
                        lowBarTimer.resetTimer();
                        lowBarState = 2;
                    }
                    break;
                case 2:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeAtIntakeSlides) < 40) {
                        outtakeSystem.setWristPos(Constants.Outtake.preClipSpecimenWristLow);
                        outtakeSystem.setArmPos(Constants.Outtake.lowSpecimenArm);
                        lowBarTimer.resetTimer();
                        lowBarState = 3;
                    }
                    break;
                case 3:
                    if (lowBarTimer.getElapsedTimeSeconds() > 0.1) {
                        if (stateMachine.getAutoPresets()) {
                            outtakeSystem.placePos(PlacePosEnum.preClipLowSpecimenAuto);
                        } else {
                            outtakeSystem.setVSlidePos(Constants.Outtake.lowSpecimenSlides);
                        }
                        lowBarTimer.resetTimer();
                        lowBarState = 4;
                    }
                    break;
                case 4:
                    if (lowBarTimer.getElapsedTimeSeconds() > 0.1) {
                        robotState.whereAmI = PlacePosEnum.lowSpecimen;
                        lowBarState = -1;
                        stateMachine.finishLowSpecimen();
                    }
                    break;
            }
        }



        if (stateMachine.doHighSpecimen()) {
            if (stateMachine.getAutoPresets()) {
                outtakeSystem.placePos(PlacePosEnum.preClipHighSpecimenAuto);
            } else {
                outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            }
            robotState.whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            switch (lowBasketState) {
                case -1:
                    outtakeSystem.setVSlidePos(Constants.Outtake.lowBasketSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setWristPos(Constants.Outtake.straightWrist);
                    robotState.whereAmI = PlacePosEnum.lowBasket;
                    lowBasketState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.lowBasketSlides) < 50) {
                        outtakeSystem.setArmPos(Constants.Outtake.safeBasketArm);
                        lowBasketTimer.resetTimer();
                        lowBasketState = 1;
                    }
                    break;
                case 1:
                    if (lowBasketTimer.getElapsedTimeSeconds() > 0.6) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                        stateMachine.finishLowBasket();
                        lowBasketState = -1;
                    }
                    break;
            }
        }

        if (stateMachine.doHighBasket()) {
            switch (highBasketState) {
                case -1:
                    outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setWristPos(Constants.Outtake.straightWrist);
                    robotState.whereAmI = PlacePosEnum.highBasket;
                    highBasketState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.highBasketSlides) < 50) {
                        outtakeSystem.setArmPos(Constants.Outtake.safeBasketArm);
                        highBasketTimer.resetTimer();
                        highBasketState = 1;
                    }
                    break;
                case 1:
                    if (highBasketTimer.getElapsedTimeSeconds() > 0.6) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                        stateMachine.finishHighBasket();
                        highBasketState = -1;
                    }
                    break;
            }
        }

        if (stateMachine.doWall()) {
            switch (wallState) {
                case -1:
                    outtakeSystem.placePos(PlacePosEnum.wall);
                    if (!robotState.hasInOutake) {
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robotState.clawToggle = false;
                    }
                    wallTimer.resetTimer();
                    wallState = 0;
                    break;
                case 0:
                    if (wallTimer.getElapsedTimeSeconds() > 0.4) {
                        robotState.whereAmI = PlacePosEnum.wall;
                        stateMachine.finishWall();
                        wallState = -1;
                    }
                    break;
            }
        }
        // AUTO PATHS *****************************************************************************~

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
            if (shakeTimer.getElapsedTimeSeconds() > 0.5) {
                if (Math.round((shakeTimer.getElapsedTimeSeconds() - 0.5) * 3) % 2 == 0 ){
                    follower.setTeleOpMovementVectors(0,0, -0.3);
                } else {
                    follower.setTeleOpMovementVectors(0,0, 0.3);
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

    public void start() {
        opmodeTimer.resetTimer();
    }

}
