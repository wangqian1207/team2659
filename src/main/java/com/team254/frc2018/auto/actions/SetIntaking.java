package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Arm;
import com.team254.frc2018.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class SetIntaking implements Action {
    private static final Intake mIntake = Intake.getInstance();

    private final boolean mWaitUntilHasCube;
    private double mStartTime;

    public SetIntaking(boolean moveToIntakingPosition, boolean waitUntilHasCube) {
        mWaitUntilHasCube = waitUntilHasCube;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    		mIntake.intake();
    }

    @Override
    public boolean isFinished() {
        if (mWaitUntilHasCube) {
            boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 2.2;
            if (timedOut) {
                System.out.println("Timed out!!!!!");
            }
            return mIntake.isLoaded() || timedOut;
        } else {
            return false;
        }
    }

    @Override
    public void done() {
        mIntake.stop();
    }
}