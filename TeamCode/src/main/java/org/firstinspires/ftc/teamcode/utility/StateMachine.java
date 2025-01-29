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
    boolean atTopSpecimen = false;

    public void resetValues() {
        goingHighSpecimen = false;
        goingLowBasket  = false;
        goingHighBasket = false;
        goingWall = false;

        goingStore = false;
        goingTransfer = false;

        hasInIntake = false;
        transferred = false;
        atStorePos = false;
        atTopSpecimen = false;
    }

    //Functions that start the movement of the robot
    public void goHighSpecimen(boolean atStorePos) {
        resetValues();
        goingHighSpecimen = true;
        this.hasInIntake = false;
        this.transferred = true;
        this.atStorePos = atStorePos;
        this.atTopSpecimen = false;
    }

    public void goLowBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingLowBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
        this.atTopSpecimen = false;
    }

    public void goHighBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingHighBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
        this.atTopSpecimen = false;
    }

    public void goWall(boolean hasInIntake, boolean transferred, boolean atStorePos, boolean atTopSpecimen) {
        resetValues();
        goingWall = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
        this.atTopSpecimen = atTopSpecimen;
    }

    public void goStore(boolean atTopSpecimen) {
        resetValues();
        goingStore = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = false;
        this.atTopSpecimen = atTopSpecimen;
    }

    public void goTransfer(boolean atStorePos, boolean atTopSpecimen) {
        resetValues();
        goingTransfer = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = atStorePos;
        this.atTopSpecimen = atTopSpecimen;
    }



    //Functions that return when we should do certain things on the robot
    public boolean doGoToStoreFromTopSpec() {
        return (goingLowBasket || goingHighBasket || goingWall || goingStore || goingTransfer) && hasInIntake && !atStorePos && atTopSpecimen;
    }

    public boolean doGoToStore() {
        return (goingLowBasket || goingHighBasket || goingWall || goingStore || goingTransfer) && hasInIntake && !atStorePos && !atTopSpecimen;
    }

    public boolean doTransfer() {
        return (goingLowBasket || goingHighBasket || goingWall || goingTransfer) && hasInIntake && atStorePos;
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
    public void finishGoToStoreFromSpec() {
        goingStore = false;

        atStorePos = true;
    }

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

    public boolean[] debug() {
        boolean[] temp = new boolean[3];
        temp[0] = hasInIntake;
        temp[1] = transferred;
        temp[2] = atStorePos;
        return temp;
    }

    public boolean isBusy() {
        return goingHighSpecimen || goingHighBasket || goingLowBasket || goingTransfer || goingStore || goingWall;
    }

}
