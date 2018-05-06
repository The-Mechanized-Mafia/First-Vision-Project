/*----v------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4567.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.usfirst.frc.team4567.grip.GripPipeline;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	public static final Subsystem m_subsystem = null;
	//Wheels
	WPI_TalonSRX LeftC= new WPI_TalonSRX(2);
	WPI_TalonSRX RightC= new WPI_TalonSRX(1);
	VictorSP LeftP= new VictorSP(1);
	VictorSP RightP= new VictorSP(0);
	
	SpeedControllerGroup L= new SpeedControllerGroup(LeftC,LeftP);
	SpeedControllerGroup R= new SpeedControllerGroup(RightC,RightP);
	
	DifferentialDrive roboDrive = new DifferentialDrive(L,R);
	//Controls
	XboxController XbC = new XboxController(0);
	Joystick leftStick = new Joystick(0);
	//Functions of Robot
	Spark shoot= new Spark(0);
	DoubleSolenoid e = new DoubleSolenoid(0,1);
	Spark horn = new Spark(1);
	//Misc Variables
	boolean rev=false;
	double tPixel;
	double distance;
	String dString;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	
    

	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);
	    camera.setFPS(30);
	   
	
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.findContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	                tPixel=r.width;
	            }
	        }
	    });
		
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	
	@Override
	public void autonomousInit() {
	
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		
		
		double centerX;
	    synchronized (imgLock) {
	        centerX = this.centerX;
	        tPixel= this.tPixel;
	    }
	    double turn = centerX - (320 / 2);
	    if(centerX<32 && centerX>-32) {
	    	roboDrive.arcadeDrive(0, 0);
	    }else {
	    roboDrive.arcadeDrive(0, turn * 0.00625);
	    DriverStation.reportError(dString, false);
	    distance= ((13/12)*320)/(2*tPixel*Math.tan(Math.toRadians(54.8/2)));
		dString= String.valueOf(distance) + "feet";
	}
	/*if(distance<1) {
		roboDrive.arcadeDrive(0.65, 0);
	} */   
	}
	
		
	
		
	

	/**
	 * This function is called periodically during operator control.
	 * The main controls
	 */
	@Override
	public void teleopPeriodic() {
		synchronized (imgLock) {
	        tPixel= this.tPixel;
	    }
		distance= ((13/12)*320)/(2*tPixel*Math.tan(Math.toRadians(54.8/2)));
		dString= "Distance to Target: " + String.valueOf(distance) + "feet";
		if(XbC.getBumperPressed(Hand.kLeft)){
			DriverStation.reportError(dString, false);
		}
		//Joystick control
		if (XbC.getStartButtonPressed()) {
		    rev = !rev;
		}

		if (rev) {
		    roboDrive.arcadeDrive(leftStick.getY(),-1*leftStick.getX());
		} else {
		    roboDrive.arcadeDrive(-1*leftStick.getY(),-1*leftStick.getX());
		}
		//Functions
		if(XbC.getAButton()){
			//DOWN
			e.set(DoubleSolenoid.Value.kReverse);
		} else if(XbC.getYButton()){
			//UP
			e.set(DoubleSolenoid.Value.kForward);
		}
		if(XbC.getTriggerAxis(Hand.kRight)>=0.5) {
			//SHOOT
			shoot.set(1);
		}
		if(XbC.getXButton()) {
			//HORN
			horn.set(1);
		}
}}
