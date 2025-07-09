package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.subsystems.PressHold;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileWriter;
import java.io.IOException;
import android.os.Environment;

import org.json.JSONTokener;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.io.*;

public class NewTeleopBlueSecret {
    //Delete me
    DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    StateMachine stateMachine;
    ElapsedTime elapsedTime = new ElapsedTime();

    //Debug Variables
    boolean debugAll = false;
    boolean debugState = false;
    boolean debugStateMachine = false;
    boolean debugPos = false;
    boolean debugClimber = false;
    boolean debugAuto = false;
    boolean debugMisc = false;

    //Auto Variables
    private final Pose frontWall  = new Pose(38, -54, Math.toRadians(90));
    private final Pose pickupWall = new Pose(38, -61, Math.toRadians(90));
    private final Pose placeSub = new Pose(0, -48, Math.toRadians(90));

    private final Pose controlPointForSubPlace = new Pose(-10, -63, 0);
    private final Pose controlPointForWall1 = new Pose(12, -50, 0);
    private final Pose controlPointForWall2 = new Pose(42, -18, 0);

    private Path toFrontWall;
    private Path frontWallToWall;
    private Path placeSubPath;

    //Input Variables
    double driveForward;
    double driveStrafe;
    double driveRotation;
    DriveSpeedEnum driveSpeed;
    boolean climbToggButton;

    double manualVSlide;
    double manualHSlide;
    double manualArm;
    double manualClimber;

    boolean sideDepo;

    boolean highSpecimen;
    boolean highBasket;
    boolean lowBasket;
    boolean frontBasket;
    boolean wallPreset;
    boolean storePos;
    boolean transfer;

    boolean clawToggleButton;

    boolean intake;
    boolean intakeColor;
    boolean unJam;

    //State Machine Variables
    boolean hasInIntake = false;;
    boolean atStorePos = false;;
    boolean hasInOuttake = false;
    boolean hasInTray = false;
    PlacePosEnum whereAmI = PlacePosEnum.wall;

    //Various Variables
    int climberStage = 0;
    Timer climberTimer = new Timer();
    int climberPos = 0;

    Timer pathTimer = new Timer();
    boolean autoMovement = false;
    boolean autoMovementOnce = true;
    boolean autoToWall = false;
    boolean autoToSub = false;
    int autoState = 0;

    boolean climberActive = false;
    boolean climbPause = false;
    boolean climbDebounce = false;
    boolean lastCurrentTrip = false;
    double lastCurrentTripTime = 2000000000;
    boolean climbStopPause = false;
    boolean climbStopPauseOnce = true;

    boolean onceTime = true;
    boolean onceState = true;
    boolean wallOnce = true;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unStoringTime = 2000000000;
    double wallTime = 2000000000;
    double lastTime = 2000000000;

    boolean transferFirstTime = true;

    boolean clawToggle = true;
    boolean clawDebounce = false;

    boolean grabbingOffWall = false;
    double grabbingOffWallTime = 2000000000;
    boolean droppingBasket = false;
    double droppingBasketTime = 2000000000;
    double sideDepoTime = 2000000000;
    boolean sideDepoFirst = false;
    boolean sideDepoDebounce = false;

    boolean sideDepoing = false;

    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean intakeingDebounce = false;
    double intakeWristTime = 2000000000;
    int extendHSlide = Constants.Intake.intakeSlidePos;

    boolean unjamming = false;
    double unjammingTime = 2000000000;
    boolean unjamAfterIntake = false;
    double unjamAfterIntakeTime = 2000000000;

    PressHold recording;
    PressHold replay;
    PressHold pointerInput;
    double lastTimer = 0;
    Pose lastPose = new Pose(0, 0, 0);
    double deltaTime = 0.1;
    double deltaError = 2;
    int replayIndex = 0;
    StateEntryJson currentReplayStates;
    PathChain replayPath;
    GamepadStateEntry gamepadDelta1;
    GamepadStateEntry gamepadDelta2;
    int logPointer = 0;


    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public NewTeleopBlueSecret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotSide = robotSide;
    }

    public void init() {
        //delete me
        motor1 = hardwareMap.get(DcMotorEx.class, "leftFront");
        motor2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        motor3 = hardwareMap.get(DcMotorEx.class, "rightBack");
        motor4 = hardwareMap.get(DcMotorEx.class, "leftBack");
        motor5 = hardwareMap.get(DcMotorEx.class, "hSlides");
        motor6 = hardwareMap.get(DcMotorEx.class, "vSlides");
        motor7 = hardwareMap.get(DcMotorEx.class, "climber");
        motor8 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //uncomment if you want telemetry on dashboard
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();


        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);

        stateMachine = new StateMachine();
        recording = new PressHold(PressHold.PressType.DoublePress);
        replay = new PressHold(PressHold.PressType.DoublePress);
        pointerInput = new PressHold(PressHold.PressType.LongPress);


        //Building paths
        toFrontWall = new Path(new BezierCurve(new Point(placeSub), new Point(controlPointForWall1), new Point(controlPointForWall2), new Point(frontWall)));
        toFrontWall.setConstantHeadingInterpolation(placeSub.getHeading());

        frontWallToWall = follower.linearPathBuilder(frontWall, pickupWall);

        placeSubPath = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointForSubPlace), new Point(placeSub)));
        placeSubPath.setConstantHeadingInterpolation(placeSub.getHeading());

        currentReplayStates = new StateEntryJson();

        loadPointer();
        telemetry.addData("pointer: ", logPointer);
        telemetry.update();
    }

    public void start() {
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap, true);
        outtakeSystem.setArmPos(Constants.Outtake.upArm);
        elapsedTime.reset();
    }

    public void recordPositions() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement" + logPointer + ".json");
        telemetry.addData("Recording, here: ", true);
        try (FileWriter writer = new FileWriter(file)) {
//            if (file.exists()) {
//                FileReader reader = new FileReader(file);
//                BufferedReader bufferedReader = new BufferedReader(reader);
//                StringBuilder content = new StringBuilder();
//                String line;
//                while ((line = bufferedReader.readLine()) != null) {
//                    content.append(line);
//                }
//                bufferedReader.close();
//                log = new JSONArray(new JSONTokener(content.toString()));
//                reader.close();
//            } else {
//            }
            Gson gson = new GsonBuilder().create();

            writer.write(gson.toJson(currentReplayStates));
            writer.close();

            telemetry.addData("Drive log written", true);
        } catch (Exception e) {
            telemetry.addData("Drive log error", e.getMessage());
        }
    }

    public void savePointer() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "pointer.json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().create();

            writer.write(gson.toJson(new PointerJson(logPointer)));
            writer.close();

            telemetry.addData("Pointer Saved written:", true);
        } catch (Exception e) {
            telemetry.addData("Pointer error", e.getMessage());
        }
    }

    public void loadPoses() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement" + logPointer + ".json");

        if (!file.exists()) return;

        StringBuilder jsonBuilder = new StringBuilder();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
            }
        } catch (Exception e) {
            telemetry.addData("Failed to Load", true);
        }

        String jsonString = jsonBuilder.toString();

        Gson gson = new GsonBuilder().create();
        currentReplayStates = gson.fromJson(jsonString, StateEntryJson.class);
    }

    public void loadPointer() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "pointer.json");

        if (!file.exists()) return;

        StringBuilder jsonBuilder = new StringBuilder();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
            }
        } catch (Exception e) {
            telemetry.addData("Failed to Load", true);
        }

        String jsonString = jsonBuilder.toString();

        Gson gson = new GsonBuilder().create();
        logPointer = gson.fromJson(jsonString, PointerJson.class).pointer;
    }

    public void loop() {
        follower.update();
        // Inputs
        debugAll = false;
        driveForward = -gamepad1.left_stick_y;
        driveStrafe = -gamepad1.left_stick_x;
        driveRotation = -gamepad1.right_stick_x;
        recording.checkStatus(gamepad1.a);
        replay.checkStatus(gamepad1.b);
        pointerInput.checkStatus(gamepad1.left_bumper);

        telemetry.addData("LOG POINTER", logPointer);

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Replay Path Null", replayPath == null);
        telemetry.addData("recording is on", recording.isOn);
        telemetry.addData("replay is on", replay.isOn);
        telemetry.addData("replay start", replay.startPress);
        telemetry.addData("Following Path: Replay Index: ", replayIndex);

        for (int i = 0; i < currentReplayStates.size; i++){
            telemetry.addData("t", (currentReplayStates.timeList.get(i)));
            telemetry.addData("pos-x", (currentReplayStates.poseList.get(i).x));
            telemetry.addData("pos-y", (currentReplayStates.poseList.get(i).y));
        }
        telemetry.addData("Size of List: ", currentReplayStates.size);

        if (pointerInput.startPress) logPointer = 0;
        if (pointerInput.isOn) logPointer = (int) Math.floor(pointerInput.time.seconds());
        if (pointerInput.endPress) savePointer();

        if(recording.startPress){
            currentReplayStates = new StateEntryJson();
            lastTimer = 0;
            lastPose = follower.getPose();
            gamepadDelta1 = new GamepadStateEntry(gamepad1);
            gamepadDelta2 = new GamepadStateEntry(gamepad2);
        }
        if (recording.isOn) {
            telemetry.addData("Error Mag: ", MathFunctions.distance(lastPose, follower.getPose()));
            if (MathFunctions.distance(lastPose, follower.getPose()) > deltaError) {
                currentReplayStates.timeList.add(recording.time.seconds());
                currentReplayStates.poseList.add(new PoseStateEntry(follower.getPose()));
                currentReplayStates.gamepad1List.add(gamepadDelta1);
                currentReplayStates.gamepad2List.add(gamepadDelta2);
                currentReplayStates.size += 1;
                lastPose = follower.getPose();
                gamepadDelta1 = new GamepadStateEntry(gamepad1);
                gamepadDelta2 = new GamepadStateEntry(gamepad2);
            }else{
                gamepadDelta1.mergeBooleans(new GamepadStateEntry(gamepad1));
                gamepadDelta2.mergeBooleans(new GamepadStateEntry(gamepad2));
            }
        }
        if (recording.endPress){
            recordPositions();
        }
        if (replay.startPress){
            loadPoses();
            if (currentReplayStates.size > 1){
                ArrayList<Path> path = new ArrayList<>();
                Pose cPos = follower.getPose();
                PoseStateEntry sPos = currentReplayStates.poseList.get(0);
                path.add(follower.linearPathBuilder(
                        new Pose(cPos.getX(), cPos.getY(), cPos.getHeading()),
                        new Pose(sPos.x, sPos.y, sPos.heading)));
                for (int i = 0; i < currentReplayStates.size - 1; i++){
                    PoseStateEntry pos1 = currentReplayStates.poseList.get(i);
                    PoseStateEntry pos2 = currentReplayStates.poseList.get(i + 1);
                    path.add(follower.linearPathBuilder(
                            new Pose(pos1.x, pos1.y, pos1.heading),
                            new Pose(pos2.x, pos2.y, pos2.heading)));
                    telemetry.addData("Pose " + i, "x: " + pos1.x + " y: " + pos1.y + " heading: " + pos1.heading);
                }
                replayPath = new PathChain(path);
                follower.followPath(replayPath);
                /*ArrayList<Point> path = new ArrayList<>();
                for (int i = 0; i < currentReplayStates.size; i++) {
                    PoseStateEntry pos = currentReplayStates.poseList.get(i);
                    path.add(new Point(new Pose(pos.x, pos.y, pos.heading)));
                }
                follower.followPath(new Path(new BezierCurve(path)));*/
            }
        }
        if (replay.isOn){
            replayIndex = follower.getCurrentPathNumber();
            gamepad1 = currentReplayStates.gamepad1List.get(replayIndex).convertToGamepad();
            gamepad2 = currentReplayStates.gamepad2List.get(replayIndex).convertToGamepad();
            follower.telemetryDebug(telemetry);
        }
        if (replay.endPress){
            follower.breakFollowing();
        }
        telemetry.addData("Hello World", true);
        //telemetry.addData("path: ", currentReplayPath);
        telemetry.addData("Replay", replay);
        if (gamepad1.right_bumper) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        climbToggButton = gamepad1.back;

        manualVSlide = -gamepad2.right_stick_y;
        if (!climberActive) {
            manualHSlide = gamepad2.right_trigger + (0.6 *(-gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger));
            if (debugAll) {
                manualHSlide = manualHSlide * 2;
            }
            manualArm = gamepad2.left_stick_y;
            manualClimber = gamepad1.right_trigger - gamepad1.left_trigger;
        } else {
            manualHSlide = 0;
            manualArm = 0;
            manualClimber = 0;
        }

        sideDepo = gamepad2.touchpad && gamepad2.touchpad_finger_1_x > 0;

        highSpecimen = gamepad2.dpad_up;
        highBasket = gamepad2.dpad_right;
        lowBasket = gamepad2.dpad_down;
        frontBasket = gamepad2.touchpad && gamepad2.touchpad_finger_1_x < 0;
        wallPreset = gamepad2.dpad_left;
        storePos = gamepad2.triangle;//y
        transfer = gamepad2.left_bumper;

        clawToggleButton = gamepad1.left_bumper || gamepad2.right_bumper;

        intake = gamepad2.cross;//a
        intakeColor = gamepad2.square;//x
        unJam = gamepad2.circle;//b

        //Drivetrain ******************************************************************************~
        if (!climberActive && !autoMovement && !climbPause && !replay.isOn) {
            if (climbToggButton) {
                climberActive = true;
                climbDebounce = true;
            }
            //Drive time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);

            if (climberActive) {
                driveTrain.setRunToPos();
                climberTimer.resetTimer();
            }
            //Autos
            if (gamepad1.x) {
                autoMovement = true;
                autoMovementOnce = true;

                autoToSub = true;
                autoToWall = false;
            }
            if (gamepad1.y) {
                autoMovement = true;
                autoMovementOnce = true;

                autoToSub = false;
                autoToWall = true;
            }
        }

        //Manual Movements ************************************************************************~
        intakeSystem.manualHSlide(manualHSlide);
        outtakeSystem.manualVSlide(manualVSlide);
        outtakeSystem.manualArm(manualArm);


        intakeSystem.update();
        outtakeSystem.update(Math.abs(manualVSlide) > 0.05 || outtakeSystem.overAmp());

        //Presets
        //Button to State Machine class ***********************************************************~
        if (highSpecimen) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeSystem.storePos();
            stateMachine.goHighSpecimen(atStorePos);
        }
        if (lowBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goLowBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (highBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goHighBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (frontBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goFrontBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (wallPreset) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeSystem.storePos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goWall(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (storePos) {
            onceTime = true;
            stateMachine.resetValues();
            intakeSystem.storePos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goStore();
            extendHSlide = Constants.Intake.intakeSlidePos;
        }
        if (transfer) {
            if (!stateMachine.doTransfer() && !stateMachine.doUnStore()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goTransfer(atStorePos);
        }


        //State Machine class to movement *********************************************************~
        if (stateMachine.doGoToStore()) {
            if (onceTime) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                clawToggle = false;
                if (whereAmI == PlacePosEnum.highSpecimen) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                } else {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                }
                intakeSystem.setHSlidePos(Constants.Intake.transferSlides);
                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 200 && onceState) {
                storeTime = elapsedTime.milliseconds();
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                onceState = false;
            }
            if (elapsedTime.milliseconds() > storeTime + 500 && elapsedTime.milliseconds() < storeTime + 600) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                atStorePos = true;
                whereAmI = PlacePosEnum.intake;
                onceState = false;
                onceTime = true;
                stateMachine.finishGoToStoreFromSpec();
            }
        }

        if (stateMachine.doTransfer()) {
            if (onceTime) {
                transferTime = elapsedTime.milliseconds();
                onceTime = false;
                transferFirstTime = true;
                if (!intakeSystem.readyToTransfer()) {
                    transferTime = elapsedTime.milliseconds() + Constants.Intake.intakeServoSpeedTime;
                }
                outtakeSystem.placePos(PlacePosEnum.intake);
            }
            if (elapsedTime.milliseconds() > transferTime + 0 && elapsedTime.milliseconds() < transferTime + 100 && transferFirstTime) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
            if (outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer() && transferFirstTime) {
                intakeSystem.setIntakePower(0);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
                hasInTray = false;
                transferFirstTime = false;
                transferTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > transferTime + 1500 && transferFirstTime) {
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
                hasInTray = true;
                stateMachine.failTransfer();
            }
            if (elapsedTime.milliseconds() > transferTime + 300 && elapsedTime.milliseconds() < transferTime + 400 && !transferFirstTime) {
                hasInOuttake = true;
                hasInIntake = false;
                onceState = true;
                onceTime = true;
                transferFirstTime = true;
                stateMachine.finishTransfer();
            }
        }

        if (stateMachine.doUnStore()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 100 && onceState) {
                atStorePos = false;
                outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                intakeSystem.setHSlidePos(0);
                unStoringTime = elapsedTime.milliseconds();
                onceState = false;
            }
            if (elapsedTime.milliseconds() > unStoringTime + 200 && elapsedTime.milliseconds() < unStoringTime + 400) {
                atStorePos = false;
                onceState = true;
                onceTime = true;
                stateMachine.finishUnStore();
            }
        }

        if (stateMachine.doHighSpecimen()) {
            clawToggle = true;
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.lowBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.lowBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                stateMachine.finishLowBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doHighBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                stateMachine.finishHighBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doFrontBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.frontBasketArm);
                stateMachine.finishFrontBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doWall()) {
            if (onceTime) {
                outtakeSystem.placePos(PlacePosEnum.wall);
                whereAmI = PlacePosEnum.wall;
                if (!hasInOuttake) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    clawToggle = false;
                }

                intakeSystem.storePos();
                wallTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            //transferring into tray
            if (elapsedTime.milliseconds() > wallTime + 0 && elapsedTime.milliseconds() < wallTime + 100 && hasInIntake) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > wallTime + 800 && elapsedTime.milliseconds() < wallTime + 900 && hasInIntake) {
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
                hasInTray = true;
                onceTime = true;
                stateMachine.finishWall();
            }

        }


        // Claw Controls **************************************************************************~
        if (!clawToggle) {
            hasInOuttake = false;
        }
        if (clawToggleButton && !clawDebounce) {
            clawToggle = !clawToggle;
            clawDebounce = true;

            // Grab
            if (clawToggle && whereAmI == PlacePosEnum.wall) {
                grabbingOffWall = true;
                wallOnce = true;
            } else if (clawToggle) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }

            // Drop
            if (!clawToggle && (whereAmI == PlacePosEnum.lowBasket || whereAmI == PlacePosEnum.highBasket)) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingBasket = true;
                droppingBasketTime = elapsedTime.milliseconds();
            } else if (!clawToggle && whereAmI == PlacePosEnum.highSpecimen) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            } else if (!clawToggle) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
        }
        if (!clawToggleButton) {
            clawDebounce = false;
        }

        if (grabbingOffWall) {
            if ((outtakeSystem.seesWall() || !clawToggleButton) && wallOnce) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                hasInOuttake = true;

                grabbingOffWallTime = elapsedTime.milliseconds();
                wallOnce = false;
            }
            if (elapsedTime.milliseconds() > grabbingOffWallTime + 300 && !wallOnce) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                grabbingOffWall = false;
                wallOnce = true;
            }
        }

        if (droppingBasket) {
            if (clawToggleButton) {
                droppingBasketTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 300 && elapsedTime.milliseconds() < droppingBasketTime + 400) {
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 500) {
                stateMachine.goStore();
                droppingBasket = false;
            }
        }

        //Intakes *********************************************************************************~
        if (gamepad1.touchpad_finger_1) {
            extendHSlide = (int) ((gamepad1.touchpad_finger_1_x + 1)/2.0 * Constants.Intake.maxSlidePos);
            if (gamepad1.touchpad) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
        }

        if (intakeColor && !intakeingColor && !intakeingDebounce) {
            intakeSystem.setHSlidePos(extendHSlide);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeingColor = true;
            intakeing = false;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }
        if (intake && !intakeing && !intakeingDebounce) {
            intakeSystem.setHSlidePos(extendHSlide);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeingColor = false;
            intakeing = true;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }

        if (intakeing) {
            if (intakeSystem.getIntakeServoPos() > Constants.Intake.wristClear && intakeSystem.intakeUntil()) {
                if (atStorePos) {
                    stateMachine.goTransfer(true);
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(0,1,300);
                gamepad2.rumble(0,1,300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                intakeing = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        if (intakeingColor) {
            if (intakeSystem.getIntakeServoPos() > Constants.Intake.wristClear && intakeSystem.intakeUntilColor()) {
                if (atStorePos) {
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(0, 1, 300);
                gamepad2.rumble(0, 1, 300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                intakeingColor = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        //makes the intake wrist not hit the robot while coming out
        if (((intakeingColor && !intakeColor) || (intakeing && !intake)) && !hasInOuttake) {
            intakeingDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            intakeWristTime = 2000000000;
        }

        //SIDE DEPOSIT*****************************************************************************~
        if (sideDepo && !sideDepoDebounce && !intakeing && !intakeingColor && !unjamAfterIntake && !unjamming && !intakeColor && !intake) {
            sideDepoing = true;
            sideDepoFirst = true;
            sideDepoDebounce = true;
            sideDepoTime = elapsedTime.milliseconds();
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
        }
        if (!sideDepo) {
            sideDepoDebounce = false;
        }

        if (sideDepoing) {
            intakeSystem.setHSlidePos(0);
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
            if (elapsedTime.milliseconds() < sideDepoTime + 100 && sideDepoFirst) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && elapsedTime.milliseconds() < sideDepoTime + 500 && sideDepoFirst) {
                intakeSystem.setIntakePower(0);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && !sideDepo && sideDepoFirst) {
                sideDepoTime = elapsedTime.milliseconds();
                sideDepoFirst = false;
            }
            if (elapsedTime.milliseconds() < sideDepoTime + 100 && !sideDepoFirst) {
                intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && !sideDepoFirst) {
                intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                sideDepoing = false;
            }
        }

        //Unjamming intake ************************************************************************~
        if (unjamAfterIntake) {
            if (elapsedTime.milliseconds() < unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unJam = true;
            }
            if (elapsedTime.milliseconds() > unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unjamAfterIntake = false;
            }
        }

        if (unJam) {
            if (!intakeing && !intakeingColor) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            unjamming = true;
            unjammingTime = elapsedTime.milliseconds();

            intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
        }

        if (unjamming && !unJam) {
            if (!(intakeing || intakeingColor)) {
                intakeSystem.setIntakePower(0);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (intakeSystem.intakeUntil()) {
                intakeSystem.setIntakePower(0);
                unjamming = false;
            }
            if (elapsedTime.milliseconds() > unjammingTime + 600) {
                hasInIntake = false;
                intakeSystem.setIntakePower(0);
                unjamming = false;
            }
        }

        if (elapsedTime.seconds() > 90 && elapsedTime.seconds() < 91) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }

//        telemetry.addData("leftFront", motor1.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rightFront", motor2.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rightBack", motor3.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("leftBack", motor4.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("hSlides", motor5.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("vSlides", motor6.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("climber", motor7.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("intakeMotor", motor8.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("max", motor1.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS) + motor3.getCurrent(CurrentUnit.AMPS) + motor4.getCurrent(CurrentUnit.AMPS) + motor5.getCurrent(CurrentUnit.AMPS) + motor6.getCurrent(CurrentUnit.AMPS) + motor7.getCurrent(CurrentUnit.AMPS) + motor8.getCurrent(CurrentUnit.AMPS));
//
//        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }

    public static class PoseEntry
    {
        double time;
        Pose pose;
        Gamepad gamepad1State;
        Gamepad gamepad2State;

        public PoseEntry(double time, Pose pose, Gamepad gamepad1State, Gamepad gamepad2State){
            this.time = time;
            this.pose = pose;
            this.gamepad1State = gamepad1State;
            this.gamepad2State = gamepad2State;
        }
    }

    public static class GamepadStateEntry {
        public boolean a, b, x, y;
        public boolean dpad_up, dpad_down, dpad_left, dpad_right;
        public boolean left_bumper, right_bumper;
        public boolean left_stick_button, right_stick_button;
        public float left_stick_x, left_stick_y;
        public float right_stick_x, right_stick_y;
        public float left_trigger, right_trigger;

        public GamepadStateEntry(Gamepad g) {
            this.a = g.a;
            this.b = g.b;
            this.x = g.x;
            this.y = g.y;
            this.dpad_up = g.dpad_up;
            this.dpad_down = g.dpad_down;
            this.dpad_left = g.dpad_left;
            this.dpad_right = g.dpad_right;
            this.left_bumper = g.left_bumper;
            this.right_bumper = g.right_bumper;
            this.left_stick_button = g.left_stick_button;
            this.right_stick_button = g.right_stick_button;
            this.left_stick_x = g.left_stick_x;
            this.left_stick_y = g.left_stick_y;
            this.right_stick_x = g.right_stick_x;
            this.right_stick_y = g.right_stick_y;
            this.left_trigger = g.left_trigger;
            this.right_trigger = g.right_trigger;
        }

        public Gamepad convertToGamepad() {
            Gamepad g = new Gamepad();

            // Buttons
            g.a = this.a;
            g.b = this.b;
            g.x = this.x;
            g.y = this.y;
            g.dpad_up = this.dpad_up;
            g.dpad_down = this.dpad_down;
            g.dpad_left = this.dpad_left;
            g.dpad_right = this.dpad_right;
            g.left_bumper = this.left_bumper;
            g.right_bumper = this.right_bumper;
            g.left_stick_button = this.left_stick_button;
            g.right_stick_button = this.right_stick_button;

            // Joysticks
            g.left_stick_x = this.left_stick_x;
            g.left_stick_y = this.left_stick_y;
            g.right_stick_x = this.right_stick_x;
            g.right_stick_y = this.right_stick_y;

            // Triggers
            g.left_trigger = this.left_trigger;
            g.right_trigger = this.right_trigger;

            return g;
        }

        public void mergeBooleans(GamepadStateEntry other) {
            this.a &= other.a;
            this.b &= other.b;
            this.x &= other.x;
            this.y &= other.y;

            this.dpad_up &= other.dpad_up;
            this.dpad_down &= other.dpad_down;
            this.dpad_left &= other.dpad_left;
            this.dpad_right &= other.dpad_right;

            this.left_bumper &= other.left_bumper;
            this.right_bumper &= other.right_bumper;

            this.left_stick_button &= other.left_stick_button;
            this.right_stick_button &= other.right_stick_button;

            // Floats remain unchanged
        }
    }

    public static class PoseStateEntry {
        public double x, y, heading;

        public PoseStateEntry(Pose pose) {
            this.x = pose.getX();
            this.y = pose.getY();
            this.heading = pose.getHeading();
        }
    }

    public static class StateEntryJson {
        public int size = 0;
        public List<Double> timeList = new ArrayList<>();
        public List<PoseStateEntry> poseList = new ArrayList<>();
        public List<GamepadStateEntry> gamepad1List = new ArrayList<>();
        public List<GamepadStateEntry> gamepad2List = new ArrayList<>();
    }

    public static class PointerJson {
        public int pointer = 0;

        public PointerJson(int pointer){
            this.pointer = pointer;
        }
    }
}