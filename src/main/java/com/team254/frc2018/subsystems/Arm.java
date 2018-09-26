package com.team254.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;

public class Arm extends Subsystem {
	private static Arm mInstance;
	public final TalonSRX mArmMaster, mArmSlave;
	private int mOffset = 0;
	private final int mPositionControl = 0;
	private final int mMotionMagicControl = 1;
	
	public Arm() {
		mArmMaster = new TalonSRX(6);
		mArmMaster.setInverted(true);
	    mArmMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	    mArmMaster.setNeutralMode(NeutralMode.Brake);
	    mArmMaster.configClosedloopRamp(0.2, 0);
	    mArmMaster.configPeakOutputForward(1, 0);
	    mArmMaster.configPeakOutputReverse(-0.75, 0);
	    mArmMaster.setSensorPhase(true);
		mArmMaster.config_kP(mPositionControl, 0.25, 0);
		mArmMaster.config_kI(mPositionControl, 0, 0);
		mArmMaster.config_kD(mPositionControl, 1.0, 0);
		mArmMaster.config_kF(mPositionControl, 0.01, 0);
		/*mArmMaster.config_kP(mMotionMagicControl, 3.0, 0);
		mArmMaster.config_kI(mMotionMagicControl, 0, 0);
		mArmMaster.config_kD(mMotionMagicControl, 20.0, 0);
		mArmMaster.config_kF(mMotionMagicControl, 1, 0);
		mArmMaster.configMotionCruiseVelocity(5000);
		mArmMaster.configMotionAcceleration(5000);*/
		mArmMaster.selectProfileSlot(mPositionControl, 0);
		
		mArmSlave = new TalonSRX(7);
		mArmSlave.setInverted(true);
	    mArmSlave.setNeutralMode(NeutralMode.Brake);
	    mArmSlave.set(ControlMode.Follower, 6);
	}
	
    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }
	
	public void set(double percentOutput) {
		mArmMaster.set(ControlMode.PercentOutput, percentOutput); //have to convert degree to revolution i guess
	}

	public void setStage(int stage) {
		if (stage == 0)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-84 + mOffset)); 
		else if (stage == 1)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-75 + mOffset));
		else if (stage == 2)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-36 + mOffset));
		else if (stage == 3)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-50 + mOffset));//portal
		else if (stage == 4)
			mArmMaster.set(ControlMode.Position, degreeToTicks(0 + mOffset));
		else if (stage == 5)
			mArmMaster.set(ControlMode.Position, degreeToTicks(10 + mOffset));
		else if (stage == 6)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-63 + mOffset)); // second deck
		else if (stage == 7)
			mArmMaster.set(ControlMode.Position, degreeToTicks(-22 + mOffset)); // farther shooting dis
		else if (stage == 8)
			mArmMaster.set(ControlMode.Position, degreeToTicks(25 + mOffset));	// backward scale
	}
	
	public void setOffset(int degree) {
		mOffset += degree;
	}
	
	public double getArmPosition() {
		return ticksToDegree(mArmMaster.getSelectedSensorPosition(0));
	}
	public boolean isFinished(int target) {
		if (Math.abs(mArmMaster.getSelectedSensorPosition(0)-degreeToTicks(target)) <= degreeToTicks(5)) {
			return true;
		}
		else
			return false;
	}
	private int degreeToTicks(int degree) {
		return degree*239; //239 = 4096*21/360
	}
	private double ticksToDegree(int ticks) {
		return ticks/239;
	}
	
    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    		mArmMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
    		mArmMaster.setSelectedSensorPosition(0,0,0);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override
            public void onLoop(double timestamp) {
                synchronized (Arm.this) {
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

