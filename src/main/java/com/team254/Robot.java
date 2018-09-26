package com.team254;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeExecutor;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private TortoDriveHelper mCheesyDriveHelper = new TortoDriveHelper();
    private AutoFieldState mAutoFieldState = AutoFieldState.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private Compressor mCompressor = new Compressor();
    private DriveControlState mDriveControlState;

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drive.getInstance(),
                    Arm.getInstance(),
                    Intake.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Arm mArm = Arm.getInstance();
    private Intake mIntake = Intake.getInstance();
    //private Climber mClimber = Climber.getInstance();
    private Joystick mDriveStick = new Joystick(0);
    private Joystick mOperatorStick = new Joystick(1);
    
    private AutoModeExecutor mAutoModeExecutor;
    
    private boolean mIfOffset = true;
    private boolean mIfShoot = true;
    private boolean mIfAutoClimb = true;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            //init camera stream
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
            camera.setExposureAuto();
            camera.setResolution(360, 270);

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mDrive.setBrakeMode(true);
            try {

            } catch (Throwable t) {
                t.printStackTrace();
                throw t;
            }
            mArm.zeroSensors();
            mTrajectoryGenerator.generateTrajectories();

            mAutoModeSelector.updateModeCreator();

            // Set the auto field state at least once.
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            mCompressor.stop();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();

            mAutoModeExecutor.start();

            mEnabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mCompressor.start();
            mCompressor.setClosedLoopControl(true);
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mAutoFieldState.disableOverride();
            
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            //mDrive.checkSystem();
            //mIntake.checkSystem();
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();
            
            // Poll FMS auto mode info and update mode creator cache
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
            mAutoModeSelector.updateModeCreator();

            if (mAutoFieldState.isValid()) {
                Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode(mAutoFieldState);
                if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                    System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                    mAutoModeExecutor.setAutoMode(autoMode.get());
                }
                System.gc();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");
        double throttle = mDriveStick.getRawAxis(1);
        double turn = mDriveStick.getRawAxis(2);

        try {
            if (mDriveStick.getRawButton(2)) {
                if (mDriveControlState != DriveControlState.PORTAL) {
                    AutoModeBase autoMode = mAutoModeSelector.getPortalShuffleMode(DriverStation.getInstance().getGameSpecificMessage().charAt(2)=='L');
                    mAutoModeExecutor.setAutoMode(autoMode);
                    mAutoModeExecutor.start();
                    mDriveControlState = DriveControlState.PORTAL;
                }
            }
            else if (mDriveControlState == DriveControlState.OPEN_LOOP || Math.abs(throttle) > 0.05 || Math.abs(turn) > 0.05){
                if (mDriveControlState != DriveControlState.OPEN_LOOP) {
                    mDriveControlState = DriveControlState.OPEN_LOOP;
                    mAutoModeExecutor.stop();
                }
                double throttle1 = mDriveStick.getRawAxis(1);
                double turn1 = mDriveStick.getRawAxis(2);
                mDrive.setOpenLoop(mCheesyDriveHelper.tortoDrive(-throttle1, -turn1, true, mDrive.isHighGear()));
            
                if (mDriveStick.getRawButton(7))
                    mDrive.setHighGear(false);
                else if (mDriveStick.getRawButton(8))
                    mDrive.setHighGear(true);
                
                if (mOperatorStick.getRawButton(8)) {
                    mIntake.intake();
                    if (mArm.getArmPosition() < -71)
                        mArm.setStage(0);
                } else if (!mOperatorStick.getRawButton(8) && mArm.getArmPosition() < -81) {
                        mArm.setStage(1);
                        mIntake.stop();
                } else if (mOperatorStick.getRawButton(7)) {
                        if (mIfShoot) {
                            mIfShoot = false;
                            mIntake.shoot();
                        }
                } else {
                        mIntake.stop();
                        mIfShoot = true;
                }
                
                if (mOperatorStick.getRawButton(2))
                        mArm.setStage(1);
                else if (mOperatorStick.getRawButton(1))
                        mArm.setStage(3);
                else if (mOperatorStick.getRawButton(3))
                        mArm.setStage(2);
                else if (mOperatorStick.getRawButton(4))
                        mArm.setStage(4);
                
                if (mOperatorStick.getPOV() == 0) {
                        if (mIfOffset) {
                            mIfOffset = false;
                            mArm.setOffset(5);
                        }
                } else if (mOperatorStick.getPOV() == 180) {
                        if (mIfOffset) {
                            mIfOffset = false;
                            mArm.setOffset(-5);
                        }
                } else
                        mIfOffset = true;
            }
            
            /*if (mDriveStick.getRawButton(5))
            		mClimber.climbDown();
            else if (mDriveStick.getRawButton(6))
            		mClimber.climbUp();
            else if (mOperatorStick.getRawButton(5) && mOperatorStick.getRawButton(6) && mIfAutoClimb) {
            		mClimber.autoClimb();
            		mIfAutoClimb = false;
            } else
            		mClimber.stop();*/
            
            outputToSmartDashboard();
        
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        Intake.getInstance().outputTelemetry();
        mAutoFieldState.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }

    private enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PORTAL
    }
}
