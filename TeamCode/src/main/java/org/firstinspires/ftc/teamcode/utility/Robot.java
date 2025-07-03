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

    private int lowBasketState = -1;

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
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    robotState.clawToggle = true;
                    if (robotState.whereAmI == PlacePosEnum.highSpecimen) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    }
                    if (intakeSystem.getHSlidePos() < Constants.Intake.transferSlides || !inAuto) {
                        intakeSystem.storeOutPos();
                    }
                    storeState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 250) {
                        outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        storeTimer.resetTimer();
                        storeState = 1;
                    }
                    break;
                case 1:
                    if (storeTimer.getElapsedTimeSeconds() > timeToWait) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        storeTimer.resetTimer();
                        storeState = 2;
                    }
                    break;
                case 2:
                    if (storeTimer.getElapsedTimeSeconds() > 0.2) {
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robotState.clawToggle = false;
                        storeTimer.resetTimer();
                        storeState = 3;
                    }
                    break;
                case 3:
                    if (storeTimer.getElapsedTimeSeconds() > 0.1) {
                        robotState.atStorePos = true;
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
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    robotState.clawToggle = false;
                    if (!intakeSystem.readyToTransfer()) {
                        transferTimer = new Timer();
                    }
                    outtakeSystem.placePos(PlacePosEnum.intake);
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    robotState.clawToggle = false;
                    transferTimer.resetTimer();
                    transferState = 0;
                    break;
                case 0:
                    if (outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer() && transferTimer.getElapsedTimeSeconds() > 0.15) {
                        // Continue
                        transferTimer.resetTimer();
                        transferState = 2;
                    }
                    if (!inAuto && transferTimer.getElapsedTimeSeconds() > 2) {
                        // Fail
                        intakeSystem.setIntakePower(0);
                        transferState = -1;
                        robotState.hasInIntake = false;
                        stateMachine.failTransfer();
                    }
                    if (inAuto && transferTimer.getElapsedTimeSeconds() > 0.75) {
                        // Auto continue
                        transferTimer.resetTimer();
                        transferState = 2;
                    }
                    break;
                case 2:
                    if (transferTimer.getElapsedTimeSeconds() > 0.05) {
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        robotState.clawToggle = true;
                        transferTimer.resetTimer();
                        transferState = 3;
                    }
                    break;
                case 3:
                    if (transferTimer.getElapsedTimeSeconds() > 0.3) {
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
                    unStoringTimer.resetTimer();
                    unStoringState = 0;
                    break;
                case 0:
                    if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 100) {
                        robotState.atStorePos = false;
                        outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        intakeSystem.setHSlidePos(0);
                        unStoringTimer.resetTimer();
                        unStoringState = 1;
                    }
                    break;
                case 1:
                    if (unStoringTimer.getElapsedTimeSeconds() > 0.2 && unStoringTimer.getElapsedTimeSeconds() < 0.4) {
                        robotState.atStorePos = false;
                        robotState.whereAmI = PlacePosEnum.highSpecimen;
                        stateMachine.finishUnStoreFromIntake();
                        unStoringState = -1;
                    }
                    break;
            }
        }

        if (stateMachine.doLowSpecimen()) {
            //todo
        }

        if (stateMachine.doUnStoreFromLowBar()) {
            //todo
        }

        if (stateMachine.doHighSpecimen()) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            robotState.whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            switch (lowBasketState) {
                case -1:
                    outtakeSystem.setVSlidePos(Constants.Outtake.lowBasketSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    robotState.whereAmI = PlacePosEnum.lowBasket;
                    lowBasketState = 0;
                    break;
                case 0:
                    if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArmHigh);
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
                    robotState.whereAmI = PlacePosEnum.highBasket;
                    highBasketState = 0;
                    break;
                case 0:
                    if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArmHigh);
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
