package org.firstinspires.ftc.teamcode.utility;

public class StateMachine {
    boolean goingHighSpecimen = false;
    boolean goingLowBasket  = false;
    boolean goingHighBasket = false;
    boolean goingFrontBasket = false;
    boolean goingWall = false;

    boolean goingStore = false;
    boolean goingTransfer = false;

    boolean hasInIntake = false;
    boolean transferred = false;
    boolean atStorePos = false;

    public void resetValues() {
        goingHighSpecimen = false;
        goingLowBasket   = false;
        goingHighBasket  = false;
        goingFrontBasket = false;
        goingWall = false;

        goingStore = false;
        goingTransfer = false;

        hasInIntake = false;
        transferred = false;
        atStorePos = false;
    }

    //Functions that start the movement of the robot
    public void goHighSpecimen(boolean atStorePos) {
        resetValues();
        goingHighSpecimen = true;
        this.hasInIntake = false;
        this.transferred = true;
        this.atStorePos = atStorePos;
    }
    public void goLowBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingLowBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }

    public void goHighBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingHighBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }
    public void goFrontBasket(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingFrontBasket = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }


    public void goWall(boolean hasInIntake, boolean transferred, boolean atStorePos) {
        resetValues();
        goingWall = true;
        this.hasInIntake = hasInIntake;
        this.transferred = transferred;
        this.atStorePos = atStorePos;
    }

    public void goStore() {
        resetValues();
        goingStore = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = false;
    }

    public void goTransfer(boolean atStorePos) {
        resetValues();
        goingTransfer = true;
        this.hasInIntake = true;
        this.transferred = false;
        this.atStorePos = atStorePos;
    }



    //Functions that return when we should do certain things on robot
    public boolean doGoToStore() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingStore || goingTransfer) && hasInIntake && !atStorePos;
    }

    public boolean doTransfer() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingTransfer) && hasInIntake && atStorePos;
    }

    public boolean doUnStore() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingHighSpecimen) && !hasInIntake && atStorePos || (goingWall && atStorePos);
    }

    public boolean doHighSpecimen() {
        return goingHighSpecimen && !atStorePos;
    }

    public boolean doLowBasket() {
        return goingLowBasket   && (!hasInIntake || transferred) && !atStorePos;
    }

    public boolean doHighBasket() {
        return goingHighBasket  && (!hasInIntake || transferred) && !atStorePos;
    }

    public boolean doFrontBasket() {
        return goingFrontBasket && (!hasInIntake || transferred) && !atStorePos;
    }

    public boolean doWall() {
        return goingWall       && !atStorePos;
    }



    //Functions to finish one thing and start the next
    public void finishGoToStoreFromSpec() {
        goingStore = false;

        atStorePos = true;
    }

    public void finishTransfer() {
        goingTransfer = false;

        hasInIntake = false;
        transferred = true;
    }

    public void failTransfer() {
        resetValues();
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
    public void finishFrontBasket() {
        goingFrontBasket = false;
    }

    public void finishWall() {
        goingWall = false;
    }

    public boolean goingHighBasket() {
        return goingHighBasket;
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
