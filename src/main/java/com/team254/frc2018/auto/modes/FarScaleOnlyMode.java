package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

import java.util.Arrays;

public class FarScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    private DriveTrajectory mSideStartToFarScale;
    private DriveTrajectory mFarScaleToFarFence;
    private DriveTrajectory mFarFenceToFarScale;
    private DriveTrajectory mFarScaleToFarFence2;
    private DriveTrajectory mFarFence2ToFarScale;

    private double mFarFenceWaitTime, mFarFence2WaitTime;

    public FarScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft), true);
        mFarScaleToFarFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft));
        mFarFenceToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFenceToFarScale.get(mStartedLeft));
        mFarScaleToFarFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence2.get(mStartedLeft));
        mFarFence2ToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFence2ToFarScale.get(mStartedLeft));

        mFarFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft).getLastState().t() - 0.1;
        mFarFence2WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft).getLastState().t() - 0.1;
        }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // System.out.println("Running far scale only");

        runAction(new WaitAction(1.0));

        // Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(1.0),
                                        new SetIntaking(false, false)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, 150.0), new Translation2d
                                                (260, 200.0), mStartedLeft),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, 150.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kUseKickstand ? 0.25 : 0.0),
                                        mFarScaleToFarFence
                                )
                        ),
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mFarFenceWaitTime)
                        ))
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFenceToFarScale,
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
                        mFarScaleToFarFence2,
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mFarFence2WaitTime)
                        ))
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFence2ToFarScale,
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
    }
}
