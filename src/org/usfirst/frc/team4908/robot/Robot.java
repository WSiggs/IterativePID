package org.usfirst.frc.team4908.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{	
    CANTalon frontLeft;
	CANTalon frontRight;
	CANTalon backLeft;
	CANTalon backRight;
		
	Joystick stick1;
    Joystick stick2;
	
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	// motors will take an input from [-1, 1], so that will be standard for all numbers
	
	double THEO_MAX; // max speed in radians per second (13 feet per second)
	double target = 0;
	double speedLeft = 0;
	double speedRight = 0;
	double rotation;
	
	// Error values
	double errorLeft;
	double errorRight;
	double totalLeft = 0;
	double totalRight = 0;
	double lastErrorLeft = 0;
	double lastErrorRight = 0;

	// PID Constants
	// Start with only P value test and evaluate overshoot, add I value if needed, add D value if needed
	double lP = 2;
	double lI = 0.0;
	double lD = 0.0;
	double rP = 1.29;
	double rI = 0.006;
	double rD = 0.790;

	// gain value
	double PGainLeft;
	double IGainLeft;
	double DGainLeft;
	double gainLeft;
	
	int count;
	
	double PGainRight;
	double IGainRight;
	double DGainRight;
	double gainRight;
	
	double deadX;
	double deadRot;
	
	boolean isDriving = false;
	RobotDrive drive;
	
	double leftTotal;
	double rightTotal;
	
	int leftSamples = 64;
	int rightSamples = 64;
	
	double maxLeft = 8.8;
	double maxRight = 9.6;
	
	public Robot() 
    {

    }
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	frontLeft = new CANTalon(1);
    	frontRight = new CANTalon(2);
    	backLeft = new CANTalon(4);
    	backRight = new CANTalon(3);        
        
        stick1 = new Joystick(0);
        stick2 = new Joystick(1);
        
        leftEncoder = new Encoder(3, 2, true);
        leftEncoder.setDistancePerPulse((2*Math.PI) / 1440); // radians per pulse
        leftEncoder.setSamplesToAverage(64);
        rightEncoder = new Encoder(7, 6, false);
        rightEncoder.setDistancePerPulse((2*Math.PI) / 1440);
        rightEncoder.setSamplesToAverage(64);
        
    	drive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
    	
    	
		SmartDashboard.putNumber("target", target);
		SmartDashboard.putNumber("SpeedLeft", speedLeft);
		SmartDashboard.putNumber("SpeedRight", speedRight); 
		SmartDashboard.putNumber("lP", lP);
		SmartDashboard.putNumber("lI", lI);
		SmartDashboard.putNumber("lD", lD);
		SmartDashboard.putNumber("rP", rP);
		SmartDashboard.putNumber("rI", rI);
		SmartDashboard.putNumber("rD", rD);
		SmartDashboard.putNumber("gainLeft", gainLeft);
		SmartDashboard.putNumber("gainRight", gainRight);
    	SmartDashboard.putNumber("LeftSamples", leftSamples);
    	SmartDashboard.putNumber("RightSamples", rightSamples);
    	
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	if(!isInDeadZone() && !isDriving)
    	{
    		isDriving = true;
    		drive();
    	}

    }
    
    public void drive()
    {    	    
    	count = 0;
    	
    	lP = SmartDashboard.getNumber("lP");
    	lI = SmartDashboard.getNumber("lI");
    	lD = SmartDashboard.getNumber("lD");
    	rP = SmartDashboard.getNumber("rP");
    	rI = SmartDashboard.getNumber("rI");
    	rD = SmartDashboard.getNumber("rD");
    	leftSamples = (int) SmartDashboard.getNumber("LeftSamples");
    	rightSamples = (int) SmartDashboard.getNumber("RightSamples");
    	
    	leftEncoder.setSamplesToAverage(leftSamples);
    	rightEncoder.setSamplesToAverage(rightSamples);
    	
    	while(!isInDeadZone())
    	{	
    		
    		if (speedLeft > maxLeft)
    				maxLeft = speedLeft;
    		if (speedRight > maxRight)
    				maxRight = speedRight;
    		
    		count++;
    		if(count % 10 == 0)
    		{
    		target = stick1.getRawAxis(1);
    		    		
    		speedLeft  = leftEncoder.getRate()  / maxLeft; // will give a number from [-1, 1]
    		speedRight = rightEncoder.getRate() / maxRight;
    		
    		errorLeft = target - speedLeft;
    		errorRight = target - speedRight;
    	
    		PGainLeft = errorLeft * lP;
    		PGainRight = errorRight * rP;

    		totalLeft += errorLeft;
    		totalRight += errorRight;
    	
    		IGainLeft = totalLeft * lI;
    		IGainRight = totalRight * rI;
    		
    		if(totalRight > 50)
    			totalRight = 0;
    		if(totalRight < -50)
    			totalRight = 0;
    		
    		if(totalLeft > 50)
    			totalLeft = 0;
    		if(totalLeft < -50)
    			totalLeft = 0;
    	
    		
    		if(IGainLeft > 1)
    			IGainLeft = 1;
    		else if(IGainLeft < -1)
    			IGainLeft = -1;
    	
    		if(IGainRight > 1)
    			IGainRight = 1;
    		else if(IGainRight < -1)
    			IGainRight = -1;
			
    		DGainLeft = (errorLeft - lastErrorLeft) * lD;
    		DGainRight = (errorRight - lastErrorRight) * rD;
    	
    		gainLeft = PGainLeft + IGainLeft + DGainLeft;
    		gainRight = PGainRight + IGainRight + DGainRight;
    	
    		//rotation = -stick2.getRawAxis(2);
    	
    		gainLeft = speedLeft + gainLeft;
    		gainRight = speedRight + gainRight;
    		
    		if(gainLeft > 1)
    			gainLeft = 1;
    		else if(gainLeft < -1)
    			gainLeft = -1;
    	
    		if(gainRight > 1)
    			gainRight = 1;
    		else if(gainRight < -1)
    			gainRight = -1;
    		 
    		
    		//moveMotors((gainLeft), (gainRight), rotation);
    	
    		drive.tankDrive(gainLeft, gainRight, false);
    		
    		SmartDashboard.putNumber("target", target);
    		SmartDashboard.putNumber("SpeedLeft", speedLeft);
    		SmartDashboard.putNumber("SpeedRight", speedRight);
    		SmartDashboard.putNumber("lP", lP);
    		SmartDashboard.putNumber("lI", lI);
    		SmartDashboard.putNumber("lD", lD);
    		SmartDashboard.putNumber("rP", rP);
    		SmartDashboard.putNumber("rI", rI);
    		SmartDashboard.putNumber("rD", rD);
    		SmartDashboard.putNumber("gainLeft", gainLeft);
    		SmartDashboard.putNumber("gainRight", gainRight);
        	SmartDashboard.putNumber("LeftSamples", leftSamples);
        	SmartDashboard.putNumber("RightSamples", rightSamples);
        	
        	System.out.println("SpeedRight\t\t" + speedRight +
        			"\nSpeedLeft\t\t" + speedLeft +
        			"\nTarget\t\t" + target
        			);
    		}
    		
    	}
    	while(!isInDeadZone() && false)
    	{
    	
    	speedLeft  = leftEncoder.getRate()  / THEO_MAX; // will give a number from [-1, 1]
    		speedRight = rightEncoder.getRate() / THEO_MAX;
    		
    	
    		drive.tankDrive(-stick1.getRawAxis(1), -stick1.getRawAxis(1), false);
    		System.out.println("speedLeft\t" + speedLeft + "\t\tspeedRight\t" + speedRight);
    	}
    	isDriving = false;
    }
    
    public void moveMotors(double speedLeft, double speedRight, double rotation)
    {

    	totalLeft = speedLeft - rotation;
    	totalRight = speedRight + rotation;
    	
    	if(totalLeft > 1)
    		totalLeft = 1;
    	else if(totalLeft < -1)
    		totalLeft = -1;
    	
    	if(totalRight > 1)
    		totalRight = 1;
    	else if(totalRight < -1)
    		totalRight = -1;
    	
    	frontLeft.set(totalLeft);
    	backLeft.set(totalLeft);
    	frontRight.set(totalRight);
    	backRight.set(totalRight);
    }
    
    public boolean isInDeadZone()
    {
    	deadX = stick1.getRawAxis(1);
    	deadRot = stick2.getRawAxis(2);
    	
    	if(deadX <= 0.05 && deadX >= -0.05)
    		if(deadRot <= 0.05 && deadRot >= -0.05)
    			return(true);
      	
    	return false;
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() 
    {
    
    }
    
}