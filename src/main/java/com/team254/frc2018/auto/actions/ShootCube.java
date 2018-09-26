package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class ShootCube implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final double kShootTime = 0.4;

    private double mStartTime;
    private boolean mActuatePiston;

    public ShootCube(double power) {
    }
    
    public ShootCube(boolean actuatePiston) {
    		mActuatePiston = actuatePiston;
    }

    public ShootCube() {
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        if (mActuatePiston)
        		mIntake.forceShoot();
        else
        		mIntake.shoot();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kShootTime;
    }

    @Override
    public void done() {
        mIntake.stop();
        mIntake.retractCylinder();
    }
}
