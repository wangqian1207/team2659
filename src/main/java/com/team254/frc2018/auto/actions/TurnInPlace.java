package com.team254.frc2018.auto.actions;

import com.team254.Constants;
import com.team254.frc2018.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;

public class TurnInPlace implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private final int mDegree;
    private double mDistance;
    private double mStartTime;

    public TurnInPlace(int degree) {
        mDegree = degree;
    }

    @Override
    public void start() {
    		mStartTime = Timer.getFPGATimestamp();
    		mDistance = Constants.kDriveWheelTrackWidthInches*Math.PI*mDegree/360;
    		mDrive.setPosition(mDistance, -mDistance);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 2;
        return mDrive.isDoneWithPosition() || timedOut;
    }

    @Override
    public void done() {
    		mDrive.stop();
    }
}