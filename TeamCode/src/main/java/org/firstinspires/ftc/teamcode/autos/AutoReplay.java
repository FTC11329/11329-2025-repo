package org.firstinspires.ftc.teamcode.autos;

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

public class AutoReplay {

    Follower follower;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

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

    public AutoReplay(Follower follower, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.follower = follower;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        recording = new PressHold(PressHold.PressType.DoublePress);
        replay = new PressHold(PressHold.PressType.DoublePress);
        pointerInput = new PressHold(PressHold.PressType.LongPress);

        currentReplayStates = new StateEntryJson();

        loadPointer();
        telemetry.addData("pointer: ", logPointer);
        telemetry.update();
    }

    public void recordPositions() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement" + logPointer + ".json");
        telemetry.addData("Recording, here: ", true);
        try (FileWriter writer = new FileWriter(file)) {
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

    public void update(){
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
            }
        }
        if (replay.isOn){
            replayIndex = (int) follower.getCurrentPathNumber();
            gamepad1 = currentReplayStates.gamepad1List.get(replayIndex).convertToGamepad();
            gamepad2 = currentReplayStates.gamepad2List.get(replayIndex).convertToGamepad();
            follower.telemetryDebug(telemetry);
        }
        if (replay.endPress){
            follower.breakFollowing();
        }
    }

    public boolean IsReplayOn(){
        return replay.isOn;
    }

    public Gamepad getGamepad1(){
        return gamepad1;
    }
    public Gamepad getGamepad2(){
        return gamepad2;
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
