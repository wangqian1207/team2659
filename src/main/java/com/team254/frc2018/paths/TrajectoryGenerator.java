package com.team254.frc2018.paths;

import com.team254.frc2018.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 130.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static final double kSimpleSwitchMaxAccel = 100.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0;

    private static final double kIntakeMaxVelocity = 95.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = new Pose2d(0.0, -2.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSideStartScalePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSideBackCube1Pose = new Pose2d(new Translation2d(198.0, 47.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kFenceCube2Intermediate = new Pose2d(new Translation2d(242.0, 80.0), Rotation2d.fromDegrees(186.0));
    public static final Pose2d kNearScaleEmptyPose = new Pose2d(new Translation2d(253.0, 28.0), Rotation2d
            .fromDegrees(10 + 180.0));
    public static final Pose2d kNearScaleFullPose = new Pose2d(new Translation2d(245.0, 28.0 + 5.0), Rotation2d.fromDegrees
            (5.0 + 180.0));

    public static final Pose2d kNearScaleFullPose1 = new Pose2d(new Translation2d(253.0, 28.0 + 8.0), Rotation2d.fromDegrees
            (10.0 + 180.0));

    public static final Pose2d kNearScaleFullPose2 = new Pose2d(new Translation2d(253.0, 28.0 + 11.0 + 7.0), Rotation2d.fromDegrees
            (20.0 + 180.0));

    public static final Pose2d kNearScaleEndPose = new Pose2d(new Translation2d(220.0, 0.0), Rotation2d.fromDegrees
            (45.0));

    public static final Pose2d kFarScaleEmptyPose = new Pose2d(new Translation2d(256.0, 200.0), Rotation2d
            .fromDegrees(-10.0 + 180.0));
    public static final Pose2d kFarScaleFullPose = new Pose2d(new Translation2d(256.0, 200.0 - 5.0), Rotation2d.fromDegrees
            (-15.0 + 180.0));
    public static final Pose2d kFarScaleFullPose1 = new Pose2d(new Translation2d(256.0, 200.0 - 8.0), Rotation2d.fromDegrees
            (-20.0 + 180.0));
    public static final Pose2d kFarScaleFullPose2 = new Pose2d(new Translation2d(256.0, 200.0 - 11.0 - 10.0), Rotation2d.fromDegrees
            (-15.0 + 180.0));

    public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-20.0, 0.0), Rotation2d.identity());

    public static final Pose2d kNearCube1Pose = new Pose2d(new Translation2d(200.0, 45.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kNearCube2Pose = new Pose2d(new Translation2d(200.0, 73.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kNearCube3Pose = new Pose2d(new Translation2d(170.0, 46.0 + 30.0 * 2 + 20.0), Rotation2d.fromDegrees(180.0 - 60.0));

    public static final Pose2d kNearFence1Pose = kNearCube1Pose;
    public static final Pose2d kNearFence2Pose = kNearCube2Pose;
    public static final Pose2d kNearFence3Pose = kNearCube3Pose.transformBy(kCenterToIntake);

    public static final Pose2d kFarCube1Pose = new Pose2d(new Translation2d(185.0, 180.0), Rotation2d.fromDegrees(180.0 + 25.0));
    public static final Pose2d kFarCube2Pose = new Pose2d(new Translation2d(183.0, 180.0 - 30.0 - 15.0), Rotation2d.fromDegrees(180.0 + 65.0));
    public static final Pose2d kFarCube3Pose = new Pose2d(new Translation2d(174.0, 180.0 - 30.0 * 2 - 20.0), Rotation2d.fromDegrees(180.0 + 60.0));

    public static final Pose2d kFarFence1Pose = kFarCube1Pose.transformBy(kCenterToIntake);
    public static final Pose2d kFarFence2Pose = kFarCube2Pose.transformBy(kCenterToIntake);
    public static final Pose2d kFarFence3Pose = kFarCube3Pose.transformBy(kCenterToIntake);

    // STARTING IN CENTER
    public static final Pose2d kSimpleSwitchStartPose = new Pose2d(0.0, -3.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSimpleSwitchLeftStartPose = new Pose2d(0.0, -17.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, -60.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(100.0, 60.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kIntermediatePose = new Pose2d(new Translation2d(20.0, 1.5), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kPyramidCubePose = new Pose2d(new Translation2d(81.0, 1.5), Rotation2d.fromDegrees(0.0))
            .transformBy(kCenterToIntake);
    public static final Pose2d kPyramidCube1Pose = new Pose2d(new Translation2d(86.0, 1.5), Rotation2d.fromDegrees(0.0))
            .transformBy(kCenterToIntake);

    public static final Pose2d kScalePoseLeft = new Pose2d(new Translation2d(253.0 + 8.0, -84.0), Rotation2d.fromDegrees(15.0 + 180.0));
    public static final Pose2d kScalePose1Left = new Pose2d(new Translation2d(253.0 + 8.0, -84.0), Rotation2d.fromDegrees(15.0 + 180.0));
    public static final Pose2d kCube1PoseLeft = new Pose2d(new Translation2d(183.0 + 6.0, -84.0 + 18.0), Rotation2d.fromDegrees(180.0 - 25.0));
    public static final Pose2d kFence1PoseLeft = kCube1PoseLeft.transformBy(kCenterToIntake);

    public static final Pose2d kScalePoseRight = new Pose2d(new Translation2d(253.0 + 8.0, -96.0), Rotation2d.fromDegrees(15.0 + 180.0));
    public static final Pose2d kScalePose1Right = new Pose2d(new Translation2d(253.0 + 8.0, -96.0), Rotation2d.fromDegrees(15.0 + 180.0));
    public static final Pose2d kCube1PoseRight = new Pose2d(new Translation2d(183.0 + 6.0, -96.0 + 18.0), Rotation2d.fromDegrees(180.0 - 25.0));
    public static final Pose2d kFence1PoseRight = kCube1PoseRight.transformBy(kCenterToIntake);

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory sideStartSimpleScale;
        public final MirroredTrajectory sideStartQuickScale;
        public final MirroredTrajectory sideStartToNearScale;
        public final MirroredTrajectory sideStartToFarScale;
        public final MirroredTrajectory sideStartToNearSwitch;
        public final MirroredTrajectory sideStartToNearBackSwitch;
        public final MirroredTrajectory sideStartToNearBackSwitch1;
        public final MirroredTrajectory sideStartToNearBackSwitch2;
        public final MirroredTrajectory sideStartToNearBackSwitch3;
        public final MirroredTrajectory sideStartToFarSwitch;
        public final MirroredTrajectory nearScaleToNearFence;
        public final MirroredTrajectory nearScaleToNearFence2;
        public final MirroredTrajectory nearScaleToNearFence3;
        public final MirroredTrajectory nearFenceToNearScale;
        public final MirroredTrajectory nearFence2ToNearScale;
        public final MirroredTrajectory nearFence3ToNearScale;
        public final MirroredTrajectory nearFence3ToEndPose;
        public final MirroredTrajectory farScaleToFarFence;
        public final MirroredTrajectory farScaleToFarFence2;
        public final MirroredTrajectory farScaleToFarFence3;
        public final MirroredTrajectory farFenceToFarScale;
        public final MirroredTrajectory farFence2ToFarScale;
        public final MirroredTrajectory farFence3ToFarScale;
        public final MirroredTrajectory switchToPortal;
        public final MirroredTrajectory portalToSwitch;

        public final Trajectory<TimedState<Pose2dWithCurvature>> simpleStartToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> simpleStartToRightSwitch;
        public final MirroredTrajectory switchToPyramidCube;
        public final MirroredTrajectory switchToCenterPyramidCube;


        private TrajectorySet() {
            sideStartSimpleScale = new MirroredTrajectory(getSideStartSimpleScale());
            sideStartQuickScale = new MirroredTrajectory(getSideStartQuickScale());
            sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());
            nearScaleToNearFence = new MirroredTrajectory(getNearScaleToNearFence());
            nearScaleToNearFence2 = new MirroredTrajectory(getNearScaleToNearFence2());
            nearScaleToNearFence3 = new MirroredTrajectory(getNearScaleToNearFence3());
            sideStartToFarScale = new MirroredTrajectory(getSideStartToFarScale());
            nearFenceToNearScale = new MirroredTrajectory(getNearFenceToNearScale());
            nearFence2ToNearScale = new MirroredTrajectory(getNearFence2ToNearScale());
            nearFence3ToNearScale = new MirroredTrajectory(getNearFence3ToNearScale());
            nearFence3ToEndPose = new MirroredTrajectory(getNearFence3ToEndPose());
            farScaleToFarFence = new MirroredTrajectory(getFarScaleToFarFence());
            farScaleToFarFence2 = new MirroredTrajectory(getFarScaleToFarFence2());
            farScaleToFarFence3 = new MirroredTrajectory(getFarScaleToFarFence3());
            farFenceToFarScale = new MirroredTrajectory(getFarFenceToFarScale());
            farFence2ToFarScale = new MirroredTrajectory(getFarFence2ToFarScale());
            farFence3ToFarScale = new MirroredTrajectory(getFarFence3ToFarScale());
            sideStartToNearSwitch = new MirroredTrajectory(getSideStartToNearSwitch());
            sideStartToNearBackSwitch = new MirroredTrajectory(getSideStartToNearBackSwitch());
            sideStartToNearBackSwitch1 = new MirroredTrajectory(getSideStartToNearBackSwitch1());
            sideStartToNearBackSwitch2 = new MirroredTrajectory(getSideStartToNearBackSwitch2());
            sideStartToNearBackSwitch3 = new MirroredTrajectory(getSideStartToNearBackSwitch3());
            sideStartToFarSwitch = new MirroredTrajectory(getSideStartToFarSwitch());

            simpleStartToLeftSwitch = getSimpleStartToLeftSwitch();
            simpleStartToRightSwitch = getSimpleStartToRightSwitch();
            switchToPyramidCube = new MirroredTrajectory(getSwitchToPyramidCube());
            switchToCenterPyramidCube = new MirroredTrajectory(getSwitchToCenterPyramidCube());

            switchToPortal = new MirroredTrajectory(getSwitchToPortal());
            portalToSwitch = new MirroredTrajectory(getPortalToSwitch());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartSimpleScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartScalePose);
            waypoints.add(kSideStartScalePose.transformBy(Pose2d.fromTranslation(new Translation2d(260.0, -20.0))));
            waypoints.add(new Pose2d(new Translation2d(300.0, 5.0), Rotation2d.fromDegrees(85.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartQuickScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartScalePose);
            waypoints.add(new Pose2d(new Translation2d(245.0, 15.0), Rotation2d.fromDegrees(20.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kSideStartPose.transformBy(Pose2d.fromTranslation(new Translation2d(-120.0, 0.0))));
            waypoints.add(kNearScaleEmptyPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(245.0, 15.0), Rotation2d.fromDegrees(20.0)));
            waypoints.add(kNearFence1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearScaleFullPose);
            waypoints.add(kNearFence2Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearScaleFullPose1);
            waypoints.add(kNearFence3Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearBackSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(new Pose2d(new Translation2d(150.0, -13.0), Rotation2d.fromDegrees(1.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, 45.0), Rotation2d.fromDegrees(168.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearBackSwitch1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(220.0, 45.0), Rotation2d.fromDegrees(168.0)));
            waypoints.add(kSideBackCube1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearBackSwitch2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideBackCube1Pose);
            waypoints.add(kFenceCube2Intermediate);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearBackSwitch3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFenceCube2Intermediate);
            waypoints.add(new Pose2d(new Translation2d(201.0, 75.0), Rotation2d.fromDegrees(185.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kIntakeMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(new Pose2d(new Translation2d(145.0, 25.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-160.0, 0.0), Rotation2d
                    .fromDegrees(0.0))));
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-220.0, -60.0), Rotation2d
                    .fromDegrees(90.0))));
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-220.0, -180.0), Rotation2d
                    .fromDegrees(90.0))));
            waypoints.add(kFarScaleEmptyPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    120.0, 140.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToFarSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(105.0, 180.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleEmptyPose);
            waypoints.add(kFarFence1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxAccel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleFullPose);
            waypoints.add(kFarFence2Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleFullPose1);
            waypoints.add(kFarFence3Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFenceToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence1Pose);
            waypoints.add(kFarScaleFullPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxAccel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFence2ToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence2Pose);
            waypoints.add(kFarScaleFullPose1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFence3ToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence3Pose);
            waypoints.add(kFarScaleFullPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFenceToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence1Pose);
            waypoints.add(kNearScaleFullPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence2ToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence2Pose);
            waypoints.add(kNearScaleFullPose1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence3ToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence3Pose);
            waypoints.add(kNearScaleFullPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence3ToEndPose() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence3Pose);
            waypoints.add(kNearScaleEndPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSimpleStartToRightSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSimpleSwitchStartPose);
            waypoints.add(kRightSwitchPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kSimpleSwitchMaxCentripetalAccel)),
                    kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSimpleStartToLeftSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSimpleSwitchLeftStartPose);
            waypoints.add(kLeftSwitchPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kSimpleSwitchMaxCentripetalAccel)),
                    kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToPyramidCube() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightSwitchPose);
            waypoints.add(kIntermediatePose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kSimpleSwitchMaxCentripetalAccel)),
                    kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
        }

        //pick up first cube
        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToCenterPyramidCube() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kIntermediatePose);
            waypoints.add(kPyramidCube1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToPortal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(30.0, -133.0), Rotation2d.fromDegrees(230.0)));
            waypoints.add(new Pose2d(new Translation2d(80.0, -90.0), Rotation2d.fromDegrees(30.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPortalToSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(80.0, -90.0), Rotation2d.fromDegrees(30.0)));
            waypoints.add(new Pose2d(new Translation2d(30.0, -133.0), Rotation2d.fromDegrees(230.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
