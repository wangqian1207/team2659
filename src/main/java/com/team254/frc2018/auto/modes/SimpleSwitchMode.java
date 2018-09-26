package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;

import java.util.Arrays;

public class SimpleSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mStartedLeft;
    private final int mSign;
    private DriveTrajectory mStartToSwitch;

    private DriveTrajectory mSwitchToIntermediate;
    private DriveTrajectory mSwitchToPyramidCube1;

    private double mStartCubeWaitTime;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        mStartedLeft = driveToLeftSwitch;

        if (mStartedLeft) {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().simpleStartToLeftSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().simpleStartToLeftSwitch.getLastState().t() - 0.75;
            mSign = -1;
        } else {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().simpleStartToRightSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().simpleStartToRightSwitch.getLastState().t() - 0.75;
            mSign = 1;
        }

        mSwitchToIntermediate = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube.get(mStartedLeft));
        mSwitchToPyramidCube1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToCenterPyramidCube.get(mStartedLeft));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");
        //Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mStartToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                	new WaitAction(0.02),
                                	new SetArm(2),
                                        new WaitAction(mStartCubeWaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

       // Go back second cube
        runAction(new SeriesAction(
                Arrays.asList(
                	mSwitchToIntermediate
                )
        ));

        // Get the second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube1,
                        new SetArm(0),
                        new SetIntaking(false, true)
                )
        ));
        runAction(new OverrideTrajectory());
        runAction(new SeriesAction(
                Arrays.asList(
                        new DriveTo(-5,-5),
                        new SetArm(7),
                        new WaitAction(0.4)
                        
                )
        ));
        //shoot second cube
        runAction(new SeriesAction(
                Arrays.asList(
                		new TurnInPlace(mSign*29),
                		new ShootCube(true),
                		new ParallelAction(
                                Arrays.asList(
                                		new TurnInPlace(mSign*-33),
                                		new SetArm(6),
                                		new WaitAction(1)
                                )
                     )
                )
        ));
        //get the third cube
        runAction(new ParallelAction(
                Arrays.asList(
                		new DriveTo(24, 24),
                		new SetIntaking(false, true)
                )
        ));
        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTo(-11,-11),
                        new SetArm(7)
                )
        ));
        //shoot the third cube
        runAction(new SeriesAction(
                Arrays.asList(
                		new TurnInPlace(mSign*28),
                		new ShootCube(true),
                		new ParallelAction(
                                Arrays.asList(
                                		new TurnInPlace(mSign*-33),
                                		new SetArm(0),
                                		new WaitAction(1)
                                )
                     )
                )
        ));
    }
}
