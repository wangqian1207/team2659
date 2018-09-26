package com.team254.frc2018.subsystems;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

public class Climber extends Subsystem {
    private static Climber mInstance = null;
    private final PWMSpeedController mClimberMaster;
    
    private Climber() {
    		mClimberMaster = new VictorSP(3);
    		mClimberMaster.setInverted(true);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void climbUp() {
    		mClimberMaster.set(1);
    }

    public void climbDown() {
    		mClimberMaster.set(-1);
    }
    
    public void autoClimb() {
    		climbUp();
    		Timer.delay(0.23);
    		climbDown();
    		Timer.delay(2.4);
    		stop();
    }
    
    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    		mClimberMaster.set(0);
    }

    @Override
    public void zeroSensors() {
    }
}