package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;

import java.util.Arrays;

public class PortalShuffleMode extends AutoModeBase {
	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mStartedLeft;
    private DriveTrajectory mToSwitch;
    private DriveTrajectory mToPortal;
    private final double mShootTime;
    private final int mSign;

    public PortalShuffleMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;

        mToSwitch = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().portalToSwitch.get(mStartedLeft), true);
        mToPortal = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().switchToPortal.get(mStartedLeft));

        mShootTime = mTrajectoryGenerator.getTrajectorySet().portalToSwitch.get(mStartedLeft).getLastState().t() - 0.1;
        mSign = robotStartedOnLeft ? 1 : -1;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
            runAction(new DriveTo(-55,-55));
            runAction(new SetArm(7));
            runAction(new TurnInPlace(mSign*190));
            runAction(new ShootCube(true));
            runAction(new TurnInPlace(mSign*170));
            runAction(new SetArm(3));
            runAction(new DriveTo(50, 50));
        /*runAction(new ParallelAction(
                Arrays.asList(
                        mToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetArm(7),
                                	new WaitAction(mShootTime),
                                	new ShootCube(true)
                                )
                        )
                )
        ));
        runAction(new SetArm(3));
        runAction(mToPortal);*/
}
}
