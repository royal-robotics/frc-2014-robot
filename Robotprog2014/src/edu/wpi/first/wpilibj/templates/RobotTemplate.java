/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    Joystick leftStick;
        Joystick rightStick;
        Joystick opperatorController;
    public void robotInit() {
        System.out.println("Robot init");
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        System.out.println("I'm in autonomos periodic");
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        System.out.println("I'm in teleop periodic");
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        System.out.println(" x = " + leftStick.getX() + "|| y = "+ leftStick.getY());
        SmartDashboard.putNumber("Speed", (Math.abs(leftStick.getX())+ Math.abs(leftStick.getY()))*50);
        SmartDashboard.putNumber("Battery",75);
        SmartDashboard.putBoolean("In Range", leftStick.getTrigger());
        SmartDashboard.putNumber("X",leftStick.getX());
        SmartDashboard.putNumber("Y",leftStick.getY());
        SmartDashboard.putNumber("Turn",rightStick.getX());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    }
    
}
