package org.firstinspires.ftc.teamcode.utility;

public class StateMachine {
    boolean goingHighSpecimen = false;
    boolean goingLowBasket  = false;
    boolean goingHighBasket = false;
    boolean goingWall = false;

    boolean goingStore = false;
    boolean goingTransfer = false;

    boolean hasInIntake = false;
    boolean transferred = false;
    boolean atStorePos = false;


    //Functions that start the movement of the robot
    public void goHighSpecimen(boolean atStorePos) {
        goingHighSpecimen = true;
        this.hasInIntake = false;
        this.transferred = true;
        this.atStorePos = atStorePos;
    }

    public void goLowBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        goingLowBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }

    public void goHighBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        goingHighBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }

    public void goWall(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        goingWall = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }

    public void goStore() {
        goingStore = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = false;
    }

    public void goTransfer(boolean atStorePos) {
        goingTransfer = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = atStorePos;
    }



    //Functions that return when we should do certain things on the robot
    public boolean doGoToStore() {
        return (goingLowBasket || goingHighBasket || goingWall || goingStore || goingTransfer) && hasInIntake && !transferred && !atStorePos;
    }

    public boolean doTransfer() {
        return (goingLowBasket || goingHighBasket || goingWall || goingTransfer) && hasInIntake && !transferred && atStorePos;
    }

    public boolean doUnStore() {
        return (goingLowBasket || goingHighBasket || goingWall || goingHighSpecimen) && !hasInIntake && atStorePos;
    }

    public boolean doHighSpecimen() {
        return goingHighSpecimen && !atStorePos;
    }

    public boolean doLowBasket() {
        return goingLowBasket  && (!hasInIntake || transferred) && !atStorePos;
    }

    public boolean doHighBasket() {
        return goingHighBasket && (!hasInIntake || transferred) && !atStorePos;
    }

    public boolean doWall() {
        return goingWall       && (!hasInIntake || transferred) && !atStorePos;
    }



    //Functions to finish one thing and start the next
    public void finishGoToStore() {
        goingStore = false;

        atStorePos = true;
    }

    public void finishTransfer() {
        goingTransfer = false;

        hasInIntake = false;
        transferred = true;
    }

    public void finishUnStore() {
        atStorePos = false;
    }

    public void finishHighSpecimen() {
        goingHighSpecimen = false;
    }

    public void finishLowBasket() {
        goingLowBasket = false;
    }

    public void finishHighBasket() {
        goingHighBasket = false;
    }

    public void finishWall() {
        goingWall = false;
    }
}
