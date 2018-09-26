package com.team254.frc2018.auto.creators;

import com.team254.AutoFieldState;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.modes.CrossAutoLineMode;
import com.team254.frc2018.auto.modes.SideStartSwitchMode;

public class SideStartSwitchModeCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private SideStartSwitchMode mGoLeftMode;
    private SideStartSwitchMode mGoRightMode;
    private CrossAutoLineMode mCrossLineMode = new CrossAutoLineMode();
    private boolean mRobotStartOnLeft;
    
    public SideStartSwitchModeCreator(boolean startOnLeft) {
    		mGoLeftMode = new SideStartSwitchMode(startOnLeft, true);
    		mGoRightMode = new SideStartSwitchMode(startOnLeft, false);
    		mRobotStartOnLeft = startOnLeft;
    }
    
    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        if (fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT && mRobotStartOnLeft)
            return mGoLeftMode;
        else if (fieldState.getOurSwitchSide() != AutoFieldState.Side.LEFT && !mRobotStartOnLeft)
            return mGoRightMode;
        else
        		return mCrossLineMode;
    }
}
