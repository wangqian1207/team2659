package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

public class Intake extends Subsystem {
    private static Intake mInstance;
    private final PWMSpeedController mIntakeMaster;
    private final Solenoid mCylinder;
    private final PowerDistributionPanel mPDP;
    private double mCurrentValue;

    private Intake() {
        mIntakeMaster = new VictorSP(0);
        mIntakeMaster.setInverted(true);
        mCylinder = new Solenoid(1);
        mPDP = new PowerDistributionPanel();
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }
    public void intake() {
		mIntakeMaster.set(1);
	}
	public void outtake() {
		if (Arm.getInstance().getArmPosition() > -15) {
			mIntakeMaster.set(-1);
		} else
			mIntakeMaster.set(-0.9);
	}
	public void shoot() {
		outtake();
		if (Arm.getInstance().getArmPosition() > -15) {
			Timer.delay(0.04);
			mCylinder.set(true);
		}
	}
	public void forceShoot() {
		outtake();
		Timer.delay(0.036);
		mCylinder.set(true);
	}
	public void retractCylinder() {
		mCylinder.set(false);
	}
	public boolean isLoaded() {
		mCurrentValue = mPDP.getCurrent(8);
		if (mCurrentValue < 50)
			return false;
		return true;
	}

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
    		mIntakeMaster.set(0);
    		retractCylinder();
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                }
            }
            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public boolean checkSystem() {
    		return true;
    }
}

