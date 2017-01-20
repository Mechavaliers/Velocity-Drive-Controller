package org.usfirst.frc.team4519.robot;

import edu.wpi.first.wpilibj.Encoder;

public class VelocityPID {

	public Encoder sensor;
	public double tickConvert;
	public double deadband, kP, kI,kD;
	
	public VelocityPID(Encoder source, float deadband, float kP, float kI, float kD, double tickConversion) {
		super();
		
		this.sensor = source;
		this.tickConvert = tickConversion;
		this.deadband = deadband;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		// TODO Auto-generated constructor stub
	}

	public Float pid(final Float target) {
        double kErr;
        double pOut;
        double iOut;
        double dOut;
        double output;

        kErr = target - (sensor.getRate() * tickConvert);

        float prevError = 0;
		float deltaError = (float) (prevError - kErr);
        prevError = (float) kErr;
        double integralError = kErr;

        if (Math.abs(target) <= deadband) {
            integralError = 0;
        }

        pOut = kErr * kP;
        iOut = integralError * kI;
        dOut = deltaError * kD;

        if (iOut > 1.0f) {
            iOut = 1.0f;
        }

        output = (pOut + iOut + dOut);

        if (output > 1.0f) {
            return 1.0f;
        }
        if (output < -1.0f) {
            return -1.0f;
        }
        return (float) output;
    }
    
    }
