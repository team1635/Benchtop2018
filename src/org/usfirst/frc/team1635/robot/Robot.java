/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1635.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private VictorSP frontLeftMotor = new VictorSP(8); 
	private VictorSP backLeftMotor = new VictorSP(1);
	private SpeedControllerGroup leftSCG = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
	
	private VictorSP frontRightMotor = new VictorSP(9);
	private VictorSP backRightMotor = new VictorSP(0);
	private SpeedControllerGroup rightSCG = new SpeedControllerGroup(frontRightMotor,backRightMotor);
	
	
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(rightSCG,leftSCG );
	private Joystick m_stick = new Joystick(0);
	private Timer m_timer = new Timer();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
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
		//next line works
		//m_robotDrive.arcadeDrive(m_stick.getY() * -1.0, m_stick.getX());
		//next line doesn't work  GetY(right) reacts to left joystick
		//m_robotDrive.tankDrive(m_stick.getY() * -1.0, m_stick.getY(GenericHID.Hand.kRight) * -1.0);
		m_robotDrive.tankDrive( m_stick.getRawAxis(5) * -1.0, m_stick.getY() * -1.0, true);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
