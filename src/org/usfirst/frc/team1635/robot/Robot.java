/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1635.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

//
//.---.  ,--.    .-----. .------.  
///_   | /  .'   /  -.   \|   ___|  
//|   |.  / -.  '-' _'  ||  '--.   
//|   || .-.  '    |_  < `---.  '. 
//|   |' \  |  |.-.  |  |.-   |  | 
//|   |\  `'  / \ `-'   /| `-'   / 
//`---' `----'   `----''  `----''  
//

public class Robot extends IterativeRobot implements PIDOutput {
	private WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(25);
	private WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(24);
	private SpeedControllerGroup leftSCG = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);

	private WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(9);
	private WPI_VictorSPX backRightMotor = new WPI_VictorSPX(0);
	private SpeedControllerGroup rightSCG = new SpeedControllerGroup(frontRightMotor, backRightMotor);

	private DifferentialDrive m_robotDrive = new DifferentialDrive(rightSCG, leftSCG);
	private Joystick m_stick = new Joystick(0);
	private Timer m_timer = new Timer();

	private String gameData;
	private AHRS ahrs;
	private PIDController turnController;
	private double rotateToAngleRate;

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;

	public Robot() {
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.7, 0.7);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		// LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		LiveWindow.add(turnController);
		turnController.setName("DriveSystem", "RotateController");
		
		//next line does not do anything, probably because ahrs does not implement Sendable
		LiveWindow.add(ahrs);  
		ahrs.setName("Navigation", "NavX");		
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		this.m_robotDrive.setSafetyEnabled(false);
		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();

		gameData = DriverStation.getInstance().getGameSpecificMessage();

		System.out.println("autonomousInit(): Game Data: " + gameData);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (m_timer.get() < 2.0) {
			//m_robotDrive.arcadeDrive(0.5, 0.0); // our robot drives backwards half speed
			m_robotDrive.arcadeDrive(-0.5, 0.0);
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		
		// myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {

			SmartDashboard.putNumber("NavX Pitch", ahrs.getPitch());
			SmartDashboard.putNumber("NavX Roll", ahrs.getRoll());
			SmartDashboard.putNumber("NavX Yaw", ahrs.getYaw());

			// next line works
			// m_robotDrive.arcadeDrive(m_stick.getY() * -1.0, m_stick.getX());
			// next line doesn't work GetY(right) reacts to left joystick
			// m_robotDrive.tankDrive(m_stick.getY() * -1.0,
			// m_stick.getY(GenericHID.Hand.kRight) * -1.0);

			boolean rotateToAngle = false;
			if (m_stick.getRawButton(5)) {  // Left bumper
				ahrs.reset();
			}
			if (m_stick.getRawButton(4)) {        // Y
				turnController.setSetpoint(0.0f);
				rotateToAngle = true;
			} else if (m_stick.getRawButton(2)) { // B
				turnController.setSetpoint(90.0f);
				rotateToAngle = true;
			} else if (m_stick.getRawButton(1)) { // A
				turnController.setSetpoint(179.9f);
				rotateToAngle = true;
			} else if (m_stick.getRawButton(3)) { // X
				turnController.setSetpoint(-90.0f);
				rotateToAngle = true;
			}

			double currentRotationRate;
			if (rotateToAngle) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
				m_robotDrive.arcadeDrive(0.0f, currentRotationRate);
			} else {
				turnController.disable();
				m_robotDrive.tankDrive(m_stick.getRawAxis(5) * -1.0, m_stick.getY() * -1.0, true);
			}
			
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		double currentRotationRate;

		currentRotationRate = rotateToAngleRate;
		m_robotDrive.arcadeDrive(0.0f, currentRotationRate);
		
		Timer.delay(0.005); // wait for a motor update time
	}
	
    @Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
}
