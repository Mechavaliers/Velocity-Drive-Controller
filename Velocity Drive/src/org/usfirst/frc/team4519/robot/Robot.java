
package org.usfirst.frc.team4519.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;



public class Robot extends IterativeRobot {

	VictorSP left = new VictorSP(1);
	VictorSP right = new VictorSP(0);
	
	Encoder leftEnc = new Encoder(0,1);
	Encoder rightEnc = new Encoder(2,3);
	
	Joystick gamepad = new Joystick(0);
	
	double ticksToFeet = (((Math.PI * 8)/ 2048)/12);
	
	VelocityPID leftController = new VelocityPID(leftEnc, 0, 0f, 0.2f, 0f, ticksToFeet);
	VelocityPID rightController = new VelocityPID(rightEnc, 0, 0f, 0.15f, 0f, ticksToFeet);
	
	public double stick;
	public double turn;
	
	double leftPower;
	double rightPower;
	
	public float constrainThrottleVelocity(double maxV, double minV, double maxOut, double minOut, double joystick){
		double slope = (maxV - minV) / (maxOut - minOut);
        double offset = minV - slope * minOut;
        return (float)(slope * joystick + offset);
	}
	
	public float constrainTurningVelocity(double maxV, double minV, double maxOut, double minOut, double joystick){
		double slope = (maxV - minV) / (maxOut - minOut);
		double offset = minV - slope * minOut;
		return (float) (slope * joystick + offset);
	}
	

    public void teleopPeriodic() {
    	
    	stick =((Math.abs(gamepad.getRawAxis(1)) > 0.1)? gamepad.getRawAxis(1): 0.0);
    	turn = ((Math.abs(gamepad.getRawAxis(4)) > 0.1)? gamepad.getRawAxis(4) : 0.0);
    	
    	SmartDashboard.putNumber("Controller Velocity Out", constrainThrottleVelocity(-13.0, 13.0, 1.0, -1.0, stick));
    	SmartDashboard.putNumber("Left Velocity", leftEnc.getRate() * ticksToFeet);
    	SmartDashboard.putNumber("Right Velocity", rightEnc.getRate() * ticksToFeet);
    	SmartDashboard.putNumber("Left Velocity Controller out", (-leftController.pid(constrainThrottleVelocity(-13.0, 13.0, 1.0, -1.0, stick))));
    	SmartDashboard.putNumber("Right Velocity Controller out", (rightController.pid(constrainThrottleVelocity(-13.0, 13.0, 1.0, -1.0, stick))));
    	
    	float leftV = (constrainThrottleVelocity(-15.0, 15.0, 1.0, -1.0, stick) - constrainTurningVelocity(-15.0, 15.0, 1.0, -1.0, turn));
    	float rightV = (constrainThrottleVelocity(-15.0, 15.0, -1.0, 1.0, stick) + constrainTurningVelocity(-15.0, 15.0, -1.0, 1.0, turn));
    	
    	leftPower = leftController.pid(leftV);
    	rightPower = rightController.pid(rightV);
    	
    	left.set(leftPower);
		right.set(rightPower);	
    }

    public void testPeriodic() {
    
    }
    
}
