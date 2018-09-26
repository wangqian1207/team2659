package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Arm;

public class SetArm implements Action {
    private static final Arm mArm = Arm.getInstance();
    private int mStage;

    public SetArm(int stage) {
        mStage = stage;
    }

    @Override
    public void start() {
    		mArm.setStage(mStage);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}