package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mSimpleScale;
    private DriveTrajectory mQuickScale;
    private DriveTrajectory mSideStartToNearScale;
    private DriveTrajectory mNearScaleToNearFence;
    private DriveTrajectory mNearFenceToNearScale;
    private DriveTrajectory mNearScaleToNearFence2;
    private DriveTrajectory mNearFence2ToNearScale;
    private DriveTrajectory mNearScaleToNearFence3;


    private double mNearFenceWaitTime, mNearFence2WaitTime, mNearFence3WaitTime, mSimpleWaitTime, mQuickWaitTime;

    public NearScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSimpleScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartSimpleScale.get(mStartedLeft), true);
        mQuickScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartQuickScale.get(mStartedLeft), true);
        mSideStartToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true);
        mNearScaleToNearFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft));
        mNearFenceToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFenceToNearScale.get(mStartedLeft));
        mNearScaleToNearFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft));
        mNearFence2ToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence2ToNearScale.get(mStartedLeft));
        mNearScaleToNearFence3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft));
        mNearFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft).getLastState().t() - 0.15 + (AutoConstants.kUseKickstand ? 0.25 : 0.0);
        mNearFence2WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft).getLastState().t() - 0.15;
        mNearFence3WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft).getLastState().t() - 0.15;
        mSimpleWaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartSimpleScale.get(mStartedLeft).getLastState().t() - 0.2;
        mQuickWaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartSimpleScale.get(mStartedLeft).getLastState().t() - 0.4;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running easy scale only");
        runAction(new ParallelAction(
                Arrays.asList(
                        mQuickScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.02),
                                        new SetArm(2),
                                        new WaitAction(mQuickWaitTime-1.7),
                                        new SetArm(4),
                                        new WaitAction(0.6),
                                        new ShootCube(true)
                                )
                        )
                )
        ));
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetArm(0),
                                        new WaitAction(1.5),
                                        new SetIntaking(false, true)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFenceToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.25),
                                        new SetArm(8),
                                        new WaitAction(1.5),
                                        new ShootCube(true)
                                )
                        )
                )
        ));

        /*runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence2,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetArm(0),
                                        new WaitAction(2),
                                        new SetIntaking(false, true)
                                )
                        )
                )
        ));
        
        // Score first cube
        /*runAction(new ParallelAction(
                Arrays.asList(
                        mSimpleScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.02),
                                        new SetArm(2),
                                        new WaitAction(mSimpleWaitTime),
                                        new SetArm(4),
                                        new WaitAction(0.6),
                                        new ShootCube(true)
                                )
                        )
                )
        ));
        runAction(new OverrideTrajectory());
        runAction(new DriveTo(-10, -10));*/

        // Get second cube
      /*  runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kUseKickstand ? 0.25 : 0.0),
                                        mNearScaleToNearFence
                                )
                        ),
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mNearFenceWaitTime)
                        ))
                )
        ));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFenceToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new ParallelAction(Arrays.asList(
                                                new WaitAction(AutoConstants.kWaitForCubeTime),
                                                new SetIntaking(false, true)
                                        )),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence2,
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mNearFence2WaitTime)
                        ))
                )
        ));
//        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFence2ToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new ParallelAction(Arrays.asList(
                                                new WaitAction(AutoConstants.kWaitForCubeTime),
                                                new SetIntaking(false, true)
                                        )),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        // Get fourth cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence3,
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mNearFence3WaitTime)
                        ))
                )
        ));*/
    }
}
