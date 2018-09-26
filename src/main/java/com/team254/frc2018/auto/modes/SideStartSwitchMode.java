package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;

import java.util.Arrays;

public class SideStartSwitchMode extends AutoModeBase {
	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mGoLeft;
    final boolean mStartedLeft;
    //private DriveTrajectory mTrajectory;
    private DriveTrajectory mFirstCube;
    private DriveTrajectory mSecondCube;
    private DriveTrajectory mShootSecondCube;
    private DriveTrajectory mThirdCube;
    private final double mShootTime;
    private final double mShootTime2;

    public SideStartSwitchMode(boolean robotStartedOnLeft, boolean switchIsLeft) {
        mStartedLeft = robotStartedOnLeft;
        mGoLeft = switchIsLeft;

        if (mGoLeft == mStartedLeft) {
            //mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearSwitch.get(mStartedLeft), true);
            mFirstCube = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearBackSwitch.get(mStartedLeft), true);
            mSecondCube = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearBackSwitch1.get(mStartedLeft));
            mShootSecondCube = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearBackSwitch2.get(mStartedLeft));
            mThirdCube = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearBackSwitch3.get(mStartedLeft));
        } else {
            //mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToFarSwitch.get(mStartedLeft), true);
            
        }
        mShootTime = mTrajectoryGenerator.getTrajectorySet().sideStartToNearBackSwitch.get(mStartedLeft).getLastState().t();
        mShootTime2 = mTrajectoryGenerator.getTrajectorySet().sideStartToNearBackSwitch2.get(mStartedLeft).getLastState().t();
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        
        runAction(new ParallelAction(
                Arrays.asList(
                        mFirstCube,
                        new SeriesAction(
                                Arrays.asList(
                                		   new WaitAction(0.02),
                                		   new SetArm(2),
                                		   new WaitAction(mShootTime),
                                		   new ShootCube(true)
                                )
                        )
                )
        ));
        runAction(new SeriesAction(
                Arrays.asList(
                        new SetArm(0),
                        new WaitAction(0.6),
                        new ParallelAction(
                                Arrays.asList(
                                        mSecondCube,
                                        new SetIntaking(false, true)
                                )
                        )   
                )
        ));
    		
    	runAction(new ParallelAction(
                    Arrays.asList(
                            mShootSecondCube,
                            new SeriesAction(
                                    Arrays.asList(
                                                new WaitAction(mShootTime2-0.1),
                                    		   new SetArm(2),
                                    		   new WaitAction(0.7),
                                    		   new ShootCube(true),
                                    		   new SetArm(0)
                                    )
                            )
                    )
            ));
    		runAction(new ParallelAction(
                    Arrays.asList(
                            mThirdCube,
                            new SeriesAction(
                                    Arrays.asList(
                                    	new SetIntaking(false, true)
                                    )
                            )
                    )
            ));
    		runAction(new OverrideTrajectory());
    		runAction(new ParallelAction(
                    Arrays.asList(
                            new DriveTo(-5,-5),
                            new SeriesAction(
                                    Arrays.asList(
                                    		   new SetArm(2),
                                    		   new WaitAction(0.7),
                                    		   new ShootCube()
                                    )
                            )
                    )
            ));
        /*runAction(new ParallelAction(
                Arrays.asList(
                        mTrajectory,
                        new SeriesAction(
                                Arrays.asList(
                                		   new WaitAction(0.02),
                                		   new SetArm(2)
                                )
                        )
                )
        ));

        runAction(new ShootCube(AutoConstants.kMediumShootPower));*/
}
}
