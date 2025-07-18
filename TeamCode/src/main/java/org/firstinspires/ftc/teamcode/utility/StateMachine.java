package org.firstinspires.ftc.teamcode.utility;

public class StateMachine {
    private boolean bringSlidesIn = true;
    boolean autoPresets = false;

    boolean goingSafeHighSpecimen = false;
    boolean goingSafeLowSpecimen = false;
    boolean goingHighSpecimen = false;
    boolean goingLowSpecimen = false;
    boolean goingLowBasket  = false;
    boolean goingHighBasket = false;
    boolean goingFrontBasket = false;
    boolean goingWall = false;

    boolean goingStore = false;
    boolean goingTransfer = false;

    boolean hasInIntake = false;
    boolean hasInOutake = false;

    boolean atStorePos = false;
    boolean atLowSpec = false;

    public void resetValues() {
        bringSlidesIn = true;
        autoPresets = false;

        goingSafeHighSpecimen = false;
        goingSafeLowSpecimen = false;
        goingHighSpecimen = false;
        goingLowSpecimen = false;
        goingLowBasket   = false;
        goingHighBasket  = false;
        goingFrontBasket = false;
        goingWall = false;

        goingStore = false;
        goingTransfer = false;

        hasInIntake = false;
        hasInOutake = false;
        atStorePos = false;
        atLowSpec = false;
    }

    //Functions that start the movement of the robot
    public void goSafeLowSpecimen(boolean atStorePos) {
        resetValues();
        goingSafeLowSpecimen = true;
        goingLowSpecimen = true;
        this.hasInIntake = false;
        this.hasInOutake = true;
        this.atStorePos = atStorePos;
    }

    public void goSafeHighSpecimen(boolean atLowSpec, boolean atStorePos) {
        resetValues();
        goingSafeHighSpecimen = true;
        goingHighSpecimen = true;
        this.hasInIntake = false;
        this.hasInOutake = true;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }
    public void goLowSpecimen(boolean atStorePos) {
        resetValues();
        goingLowSpecimen = true;
        this.hasInIntake = false;
        this.hasInOutake = true;
        this.atStorePos = atStorePos;
    }

    public void goHighSpecimen(boolean atLowSpec, boolean atStorePos) {
        resetValues();
        goingHighSpecimen = true;
        this.hasInIntake = false;
        this.hasInOutake = true;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }
    public void goLowBasket(boolean hasInIntake, boolean transferred, boolean atLowSpec, boolean atStorePos) {
        resetValues();
        goingLowBasket = true;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = transferred;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }

    public void goHighBasket(boolean hasInIntake, boolean transferred, boolean atLowSpec, boolean atStorePos) {
        resetValues();
        goingHighBasket = true;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = transferred;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }
    public void goFrontBasket(boolean hasInIntake, boolean transferred, boolean atLowSpec, boolean atStorePos) {
        resetValues();
        goingFrontBasket = true;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = transferred;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }


    public void goWall(boolean hasInIntake, boolean atLowSpec, boolean atStorePos, boolean override) {
        if (override) {
            resetValues();
        }
        goingWall = true;
        this.hasInIntake = hasInIntake;
        this.atLowSpec = atLowSpec;
        this.atStorePos = atStorePos;
    }
    public void goWall(boolean hasInIntake, boolean atLowSpec, boolean atStorePos) {
        goWall(hasInIntake, atLowSpec, atStorePos, true);
    }

    public void goStore() {
        resetValues();
        goingStore = true;
        this.hasInIntake = true;
        this.hasInOutake = false;
        this.atStorePos = false;
    }

    public void goTransfer(boolean atStorePos) {
        resetValues();
        goingTransfer = true;
        this.hasInIntake = true;
        this.hasInOutake = false;
        this.atStorePos = atStorePos;
    }



    //Functions that return when we should do certain things on robot
    public boolean doGoToStore() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingWall || goingStore || goingTransfer) && hasInIntake && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doTransfer() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingWall || goingTransfer) && hasInIntake && atStorePos;
    }

    public boolean doUnStoreFromLowBar() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingLowSpecimen || goingHighSpecimen || goingWall) && atLowSpec && false;
    }

    public boolean doUnStoreFromIntake() {
        return (goingLowBasket || goingHighBasket || goingFrontBasket || goingLowSpecimen || goingHighSpecimen || goingWall) && !hasInIntake && atStorePos;
    }

    public boolean doSafeLowSpecimen() {
        return goingSafeLowSpecimen && !goingLowSpecimen && !atStorePos;
    }

    public boolean doSafeHighSpecimen() {
        return goingSafeHighSpecimen && !goingHighSpecimen && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doLowSpecimen() {
        return goingLowSpecimen && !atStorePos;
    }

    public boolean doHighSpecimen() {
        return goingHighSpecimen && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doLowBasket() {
        return goingLowBasket   && (!hasInIntake || hasInOutake) && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doHighBasket() {
        return goingHighBasket  && (!hasInIntake || hasInOutake) && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doFrontBasket() {
        return goingFrontBasket && (!hasInIntake || hasInOutake) && !atStorePos /*&& !atLowSpec*/;
    }

    public boolean doWall() {
        return goingWall && !hasInIntake && !atStorePos /*&& !atLowSpec*/;
    }



    //Functions to finish one thing and start the next
    public void finishGoToStore() {
        goingStore = false;

        atStorePos = true;
    }

    public void finishTransfer() {
        goingTransfer = false;

        hasInIntake = false;
        hasInOutake = true;
    }

    public void failTransfer() {
        resetValues();
    }

    public void finishUnStoreFromLowBar() {
        atLowSpec = false;
    }

    public void finishUnStoreFromIntake() {
        atStorePos = false;
    }

    public void finishSafeLowSpecimen() {
        goingSafeLowSpecimen = false;
    }

    public void finishSafeHighSpecimen() {
        goingSafeHighSpecimen = false;
    }

    public void finishLowSpecimen() {
        goingLowSpecimen = false;
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
    public boolean goingWall() {
        return goingWall;
    }
    public boolean[] debug() {
        boolean[] temp = new boolean[5];
        temp[0] = hasInIntake;
        temp[1] = hasInOutake;
        temp[2] = atStorePos;
        temp[3] = atLowSpec;
        temp[4] = goingHighBasket;
        return temp;
    }

    public boolean getBringSlidesIn() {
        return bringSlidesIn;
    }

    public void setBringSlidesIn(boolean bringSlidesIn) {
        this.bringSlidesIn = bringSlidesIn;
    }

    public boolean getAutoPresets() {
        return autoPresets;
    }

    public void setAutoPresets(boolean autoPresets) {
        this.autoPresets = autoPresets;
    }

    public boolean isBusy() {
        return goingHighSpecimen || goingHighBasket || goingLowBasket || goingTransfer || goingStore || goingWall;
    }

}
