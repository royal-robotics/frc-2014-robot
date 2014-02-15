/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.DigitalIOButton;


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
    Joystick operatorController;
    
    RobotDrive mecanum;
    
    Talon frontRightDrive;
    Talon frontLeftDrive;
    Talon backRightDrive;
    Talon backLeftDrive;
    
    Talon pickup;
    Talon ratchet;
    
    Compressor pump;
    
    DoubleSolenoid pickLift;
    DoubleSolenoid ratchetLoose;
    
    Relay ledRed;
    Relay ledGreen;
    Relay ledBlue;
    
    AxisCamera camera;
    Servo cameraX;
    Servo cameraY;
    
    Gyro gyro;
    AnalogChannel ultrasonic;
    
    DigitalInput catapultStop;
    
    /* activate twerking abilities with: A
     *Robot initialization give the valuse for each object that has been made.
    move the robot at this speed: 10 MPH*/
    public void robotInit() {
        System.out.println("Robot init");
        
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        operatorController = new Joystick(3);
        
        frontRightDrive = new Talon(1);
        frontLeftDrive = new Talon(2);
        backRightDrive = new Talon(3);
        backLeftDrive = new Talon(4);
        
        mecanum = new RobotDrive(frontRightDrive, frontLeftDrive, backRightDrive, backLeftDrive);
        mecanum.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        mecanum.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        pickup = new Talon(5);
        ratchet = new Talon(6);
        
        cameraX = new Servo(7);
        cameraY = new Servo(8);
        
        pump = new Compressor(1,1);
        
        ratchetLoose = new DoubleSolenoid(1,4);        
        pickLift = new DoubleSolenoid(2,3); 
        
        ledRed = new Relay(2);
        ledGreen = new Relay(3);
        ledBlue = new Relay(4);
        
        catapultStop = new DigitalInput(3);
        gyro = new Gyro(1);
        ultrasonic = new AnalogChannel(4);
        
        //initializes the piston for the ratchet
        ratchetLoose.set(DoubleSolenoid.Value.kReverse);
        ratchetIn = true;
        pickLift.set(DoubleSolenoid.Value.kReverse);
        pickDown = false;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        System.out.println("I'm in autonomos periodic");
        System.out.println("Work in progress if you couldn't tell.");
    }

    /**
     * This function is called periodically during operator control
     */
    double dead = 0.1;
    public void teleopPeriodic() {
        System.out.println("I'm in teleop periodic");
        //Creates Dashboard
        pump.start();
        SmartDashboard.putNumber("Speed", (Math.abs(leftStick.getX())+ Math.abs(leftStick.getY()))*50);
        SmartDashboard.putNumber("Battery",75);
        SmartDashboard.putBoolean("High speed", leftStick.getTrigger());
        SmartDashboard.putNumber("X",leftStick.getX());
        SmartDashboard.putNumber("Y",leftStick.getY());
        SmartDashboard.putNumber("Turn",rightStick.getX());
        SmartDashboard.putBoolean("BTN 1", operatorController.getRawButton(1));
        SmartDashboard.putBoolean("BTN 2", operatorController.getRawButton(2));
        SmartDashboard.putBoolean("BTN 3", operatorController.getRawButton(3));
        SmartDashboard.putBoolean("BTN 4", operatorController.getRawButton(4));
        SmartDashboard.putBoolean("BTN 5", operatorController.getRawButton(5));
        SmartDashboard.putBoolean("BTN 6", operatorController.getRawButton(6));
        SmartDashboard.putBoolean("BTN 7", operatorController.getRawButton(7));
        SmartDashboard.putBoolean("BTN 8", operatorController.getRawButton(8));
        SmartDashboard.putBoolean("BTN 9", operatorController.getRawButton(9));
        SmartDashboard.putBoolean("BTN 10", operatorController.getRawButton(10));
        SmartDashboard.putBoolean("Throttle", leftStick.getBumper());
        SmartDashboard.putBoolean("Stop switch",catapultStop.get());
      
        //Calls the motor functions.
        int div;
        if(leftStick.getTrigger() == true) {
            div = 1;
        } else div=5;
        
        double leftS_X = -(leftStick.getX()/div);
        if(leftS_X > -dead &&leftS_X < dead)
        {
            leftS_X = 0;
        }
        
        double leftS_Y = leftStick.getY()/div;
        if(leftS_Y > -dead &&leftS_Y < dead)
        {
            leftS_Y = 0;
        }
        
        double rightS_X = rightStick.getX()/div;
        if(rightS_X > -dead &&rightS_X < dead)
        {
            rightS_X = 0;
        }
        mecanum.mecanumDrive_Cartesian(leftS_Y, leftS_X, rightS_X, gyro.getAngle());
        
        operator();
    }
    /*
     * Opperator controls are interpreted here.
    */ 
    int timer = 0;
    boolean ratchetIn = true;
    boolean pickDown = false;
    //double angleX = cameraX.getAngle();
    //double angleY = cameraY.getAngle();
    public void operator()
    {
        double axis1 = operatorController.getRawAxis(1);
        SmartDashboard.putNumber("axis1",axis1);
        double axis2 = operatorController.getRawAxis(2);
        SmartDashboard.putNumber("axis2",axis2);
        double axis3 = operatorController.getRawAxis(3);
        SmartDashboard.putNumber("axis3",axis3);
        double axis4 = operatorController.getRawAxis(4);
        SmartDashboard.putNumber("axis4",axis4);
        double axis5 = operatorController.getRawAxis(5);
        SmartDashboard.putNumber("axis5",axis5);
        double axis6 = operatorController.getRawAxis(6);
        SmartDashboard.putNumber("axis6",axis6);
        double axis7 = operatorController.getRawAxis(7);
        SmartDashboard.putNumber("axis7",axis7);
        double axis8 = operatorController.getRawAxis(8);
        SmartDashboard.putNumber("axis8",axis8);
        if(axis2<-dead)
        {
            pickup.set(axis2);
        }
        else if(axis2 > dead)
        {
            pickup.set(axis2);
        }
        else 
        {
            pickup.set(0);
        }
        
        if(operatorController.getRawButton(6) == true)
        {
             ratchetLoose.set(DoubleSolenoid.Value.kForward);
             ratchetIn=false;
             timer = 0;
        }
        
        if(operatorController.getRawButton(4) == true)
        {
             ratchetLoose.set(DoubleSolenoid.Value.kReverse);
             ratchetIn=true;
        }
        
        if(operatorController.getRawButton(5) == true)
        { 
            if(ratchetIn==false)
            {
                ratchetLoose.set(DoubleSolenoid.Value.kReverse);
                timer++;
                if(timer >40) ratchetIn = true;
            }else if(ratchetIn == true && catapultStop.get() == false)
            {
             ratchet.set(-0.5);
            } else ratchet.set(0);
        } else ratchet.set(0);
        System.currentTimeMillis();
        if(operatorController.getRawButton(8) == true)
        {
                pickLift.set(DoubleSolenoid.Value.kForward);
                pickDown=true;
        }
        
        if(operatorController.getRawButton(7) == true)
        {
                pickLift.set(DoubleSolenoid.Value.kReverse);
                pickDown=false;
        }
        
        /*if(operatorController.getRawAxis(6) == 1)
        {
            cameraX.set(angleX + 0.05);
        }
        
        if(operatorController.getRawAxis(6) == -1)
        {
            cameraX.set(angleX - 0.05);
        }*/
    }
    /* This function is called periodically during test mode
     */
    public void testPeriodic() {
    }
    
}