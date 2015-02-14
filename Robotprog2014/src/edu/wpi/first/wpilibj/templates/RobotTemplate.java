/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.*;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.util.Random;

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
    
    Joystick rightStick;
    Joystick leftStick;
    Joystick operatorController;
    
    RobotDrive mecanum;
    
    Talon frontRightDrive;
    Talon frontLeftDrive;
    Talon backRightDrive;
    Talon backLeftDrive;
    
    Talon pickup;
    Talon ratchet;
    
    Compressor pump;
    
    Relay ledRed;
    Relay ledGreen;
    Relay ledBlue;
    
    DoubleSolenoid pickLift;
    DoubleSolenoid ratchetLoose;
    DoubleSolenoid ballHolder;
    
    //LED Colors
    static final int RED = 0;
    static final int GREEN = 1;
    static final int BLUE = 2;
    static final int YELLOW = 3;
    static final int PURPLE = 4;
    static final int CYAN = 5;
    static final int WHITE = 6;
    static final int NOTHING = 7;
    
    AxisCamera camera;
    CriteriaCollection cc;
    
    //Minimum area of particles to be considered
    final int AREA_MINIMUM = 100;
    
    boolean hot = false;
    
    AnalogChannel ultrasonic;
    
    DigitalInput catapultStop;
    
    Encoder driveEncoder;
    Encoder ratchetEncoder;
    
    double distanceAvg[] = new double[5];
    int distanceAvgCount = 0;
    
    Random randLED;
    
    Timer autoTime;
    Timer teleTime;

    double raiseStart;
 
    public void robotInit() {
        System.out.println("Robot init");
        
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        operatorController = new Joystick(3);
        
        frontRightDrive = new Talon(1);
        frontLeftDrive = new Talon(2);
        backRightDrive = new Talon(3);
        backLeftDrive = new Talon(4);
        
        mecanum = new RobotDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);        
        
        pickup = new Talon(5);
        ratchet = new Talon(6);
        
        pump = new Compressor(1,1);
        
        ledRed = new Relay(4);
        ledGreen = new Relay(3);
        ledBlue = new Relay(2);
        
        ratchetLoose = new DoubleSolenoid(1,4);        
        pickLift = new DoubleSolenoid(2,3); 
        ballHolder = new DoubleSolenoid(5,6);
        
        randLED = new Random();
        randLED.nextInt(7);
        
        autoTime = new Timer();
        autoTime.reset();
        teleTime = new Timer();
        teleTime.reset();

        catapultStop = new DigitalInput(3);

        ultrasonic = new AnalogChannel(4);
        camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        hot = false;
        
        driveEncoder = new Encoder(6,8);
        ratchetEncoder = new Encoder(10,11);
        driveEncoder.setDistancePerPulse((2 * 3.1415 * 3.0) / 1440.0);
        driveEncoder.start();
        ratchetEncoder.start();
        
        //Initializes the Solenoid
        ratchetLoose.set(DoubleSolenoid.Value.kReverse);
        pickLift.set(DoubleSolenoid.Value.kReverse);
        ballHolder.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This function is called periodically during autonomous
     */
    double drive = (1.0/3.0);
    double curve = 0.0;
    boolean autoLaunch = false;
    boolean checkHot = false;
    boolean timeStart = false;
    boolean stop = false;
    
    public void autonomousInit() {
        autoTime.stop();
        autoTime.reset();
        autoLaunch = false;
        checkHot = false;
        hot = false;
        timeStart = false;
        stop = false;
        ballHolder.set(DoubleSolenoid.Value.kForward);
    }
    
    public void autonomousPeriodic() {
        DriverStation ds = DriverStation.getInstance();
        Watchdog watchdog = Watchdog.getInstance();
        if (leftStick.getZ() <= 1.0) {  //1 Ball, Hot Check
            //Get the Range
            double range = 0.0;
            distanceAvg[distanceAvgCount] = ultrasonic.getVoltage()*100;
            distanceAvgCount++;
            if (distanceAvgCount >= distanceAvg.length)
            {
                distanceAvgCount = 0;
            }
            for (int i = 0; i < distanceAvg.length; i++)
            {
                range = range + distanceAvg[i];
            }
            range = range / (double)distanceAvg.length;
            watchdog.feed();

            pickLift.set(DoubleSolenoid.Value.kForward);

            if (((ultrasonic.getVoltage() * 100) < 167.0 && ds.getMatchTime() > 1.6) || autoLaunch)
            {
                //Stop
                stop = true;
                watchdog.feed();
                mecanum.mecanumDrive_Polar(0, 0, 0);
                if (! checkHot)
                {
                    hot = isHot();  //Check for Hot Goal
                    watchdog.feed();
                    checkHot = true;
                    raiseStart = ds.getMatchTime();
                    ballHolder.set(DoubleSolenoid.Value.kReverse);
                }
            }
            else
            {
                //Drive
                watchdog.feed();
                mecanum.mecanumDrive_Polar((1.0/3.0), -90, 0.0);
            }

            if ((((ultrasonic.getVoltage() * 100) > 60.0 && (ultrasonic.getVoltage() * 100) < 167.0 && ds.getMatchTime() > 3.9) && ((ds.getMatchTime() - raiseStart) > 0.25) && ! autoLaunch && hot) || (ds.getMatchTime() > 7.0 && ! autoLaunch))
            {
                //Shoot
                watchdog.feed();
                ratchetLoose.set(DoubleSolenoid.Value.kForward);  // Fire
                autoLaunch = true;
            }
            dashboard();  //Update Dashboard
        }
        /*else if (rightStick.getZ() < 0.5 && rightStick.getZ() > 0.0) {  //1 Ball, No Hot Check
            mecanum.mecanumDrive_Polar(1.0/3.0, -90, 0.0);  //Drive
            pickLift.set(DoubleSolenoid.Value.kForward);
            if (ds.getMatchTime() > 1.7)
            {
                //Stop and Shoot
                watchdog.feed();
                mecanum.mecanumDrive_Polar(0, 0, 0);
                ratchetLoose.set(DoubleSolenoid.Value.kForward);
            }
            dashboard();
        }
        else if (rightStick.getZ() >= 0.5) {  //Just Drive, don't shoot
            mecanum.mecanumDrive_Polar(1.0/3.0, -90, 0.0);  //Drive
            if (ds.getMatchTime() > 1.7)
            {
                watchdog.feed();
                mecanum.mecanumDrive_Polar(0, 0, 0);  //Stop
            }
            dashboard();
        }
        else if (rightStick.getZ() <= -0.5) {  //2 Balls
            if (ds.getMatchTime() > 0.0 && ds.getMatchTime() < 1.2)
            {
                watchdog.feed();
                pickLift.set(DoubleSolenoid.Value.kForward);  //Pickup Down
            }
            
            if ((ultrasonic.getVoltage() * 100) > 208 && ds.getMatchTime() > 1.2)
            {
                watchdog.feed();
                mecanum.mecanumDrive_Polar(0.5, -90.0, 0.0);  //Drive
            }
            
            if ((ultrasonic.getVoltage() * 100) < 208 && (ultrasonic.getVoltage() * 100) > 185 && ds.getMatchTime() > 1.2)
            {
                watchdog.feed();
                ratchetLoose.set(DoubleSolenoid.Value.kForward);  //Fire
                mecanum.mecanumDrive_Polar(0.5, -90.0, 0.0);
                if (! timeStart)
                {
                    //Start Timer
                    autoTime.reset();
                    autoTime.start();
                    timeStart = true;
                }
            }
            
            if (autoTime.get() > 0.0 && (ultrasonic.getVoltage() * 100) > 100)
            {
                mecanum.mecanumDrive_Polar(0.5, -90.0, 0.0);
            }
            
            if (autoTime.get() > 0.0 && (ultrasonic.getVoltage() * 100) < 100)
            {
                mecanum.mecanumDrive_Polar(0.0, 0.0, 0.0);
            }
            
            if (autoTime.get() > 5.0)
            {
                pickup.set(0.0);
                ratchet.set(0.0);
                ratchetLoose.set(DoubleSolenoid.Value.kForward);
            }
            
            if (autoTime.get() > 0.2 && autoTime.get() < 5.0)
            {
                ratchetLoose.set(DoubleSolenoid.Value.kReverse);
                if (! catapultStop.get())
                {
                    ratchet.set(-1.0);
                }
                else
                {
                    ratchet.set(0.0);
                }
            }
            
            if (autoTime.get() > 0.8 && autoTime.get() < 2.5)
            {
                pickup.set(-0.8);
            }
            
            if (autoTime.get() > 2.5 && autoTime.get() < 2.62)
            {
                pickup.set(0.75);
            }
            
            if (autoTime.get() > 2.62 && autoTime.get() < 2.9)
            {
                pickup.set(-0.8);
            }
            
            if (autoTime.get() > 2.9)
            {
                pickup.set(0.0);
            }
            dashboard();
        }*/
    }
    
    /**
     * This function is called periodically during operator control
     */
    double dead = 0.1;
    int pickTime = 50;

    public void teleopInit() {
        pump.start();
        ratchetLoose.set(DoubleSolenoid.Value.kReverse);
        pickLift.set(DoubleSolenoid.Value.kReverse);
        teleTime.reset();
    }
    
    public void teleopPeriodic() {
        dashboard();  //Update Dashboard
        
        //Scales Driving Speed
        double div;
        if(rightStick.getTrigger()) {
            div = 3;
        } else if (leftStick.getTrigger()){
            div = 1;
        } else div = 2;
        
        Watchdog.getInstance().feed();
        
        double mag = leftStick.getMagnitude()/div;  //Magnitude of Drive
        if (mag > -dead && mag < dead)  //Dead Zone
        {
            mag = 0;
        }
        
        double deg = leftStick.getDirectionDegrees();  //Direction of Drive
        
        double rightS_X = (rightStick.getX()/div);  //Rotation of Drive
        if(rightS_X > -dead && rightS_X < dead)  //Dead Zone
        {
            rightS_X = 0;
        }
        
        Watchdog.getInstance().feed();//nerds be like: "Hi, I'm Moisey
        
        
        mecanum.mecanumDrive_Polar(rightStick.getAxis(Joystick.AxisType.kX),rightStick.getAxis(Joystick.AxisType.kX), -leftStick.getAxis(Joystick.AxisType.kY));
        /*if (mag > 0.5 && deg > -110 && deg < -70)
        {
            mecanum.mecanumDrive_Polar(mag, -90.0, rightS_X);
        }
        if (mag > 0.5 && deg > 70 && deg < 110)
        {
            mecanum.mecanumDrive_Polar(mag, 90.0, rightS_X);
        }*/
        
        operator();  //Process Operator Inputs
        Watchdog.getInstance().feed();
        setLED();  //Set LED Color
        Watchdog.getInstance().feed();
        
        if (rightStick.getRawButton(2))
        {
            hot = isHot();
            Watchdog.getInstance().feed();
        }
        
        //Use To Calibrate Talons
        if (operatorController.getRawButton(3))
        {
            frontRightDrive.set(1.0);
            frontLeftDrive.set(1.0);
            backRightDrive.set(1.0);
            backLeftDrive.set(1.0);
        }
        
        if (operatorController.getRawButton(2))
        {
            frontRightDrive.set(-1.0);
            frontLeftDrive.set(-1.0);
            backRightDrive.set(-1.0);
            backLeftDrive.set(-1.0);
        }
    }
    
    public void disabledInit() {
        dashboard(); //Update Dashboard
    }

    public void disabledPeriodic() { //fart face HAHAHA
        dashboard();  //Update Dashboard
    }

    double divTwo = 1.0;
    double axis2=0;
    boolean pickDown = false;
    public void operator()  //Process Operator Inputs
    {
        Watchdog watchdog = Watchdog.getInstance();
        
        if(operatorController.getRawAxis(5) == 1.0)
        {
            pickup.set(1.0);  //D-Pad Right Ejects Balls
        }
        else if(operatorController.getRawButton(1))
        {
            pickup.set(-1.0);  //Button 1 Sucks Balls
        }
        else 
        {
            pickup.set(0);
        }
        
        watchdog.feed();
        
        if(operatorController.getRawButton(3))
        {
             ratchetLoose.set(DoubleSolenoid.Value.kReverse);  //Button 4 Engages Ratchet
        }
        
        if(operatorController.getRawButton(6) && pickLift.get() == DoubleSolenoid.Value.kForward && ballHolder.get() == DoubleSolenoid.Value.kReverse)
        {
             ratchetLoose.set(DoubleSolenoid.Value.kForward);  //Button 6 Shoots if pickup is down and ball holder is up
        }
        
        watchdog.feed();
        
        if (operatorController.getRawButton(5) && ! catapultStop.get())
        {
            //Button 5 Brings Ratchet Down, Automatically Engages Ratchet
            ratchetLoose.set(DoubleSolenoid.Value.kReverse);
            ratchet.set(-1);
        }
        else
        {
            ratchet.set(0); //THAT SHO' IS RATCHET
        }
        
        watchdog.feed();
        
        if(operatorController.getRawButton(8))
        {
                pickLift.set(DoubleSolenoid.Value.kForward);  //Button 8 Brings Pickup Down
        }
        
        if(operatorController.getRawButton(7))
        {
                pickLift.set(DoubleSolenoid.Value.kReverse);  //Button 7 Brings Puckup Up
        }
        
        watchdog.feed();
        
        if(operatorController.getRawButton(2))
        {
            ballHolder.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            ballHolder.set(DoubleSolenoid.Value.kReverse);
        }
        
        if (operatorController.getRawButton(4))
        {
            teleTime.reset();
            teleTime.start();
        }
        
        if (teleTime.get() > 0.0 && teleTime.get() < 0.12)
            {
                pickup.set(0.75);
            }
            
        if (teleTime.get() > 0.12 && teleTime.get() < 0.4)
            {
                pickup.set(-0.8);
            }
        
        if (teleTime.get() > 0.4)
        {
            teleTime.stop();
            teleTime.reset();
        }
    }
    
    
    /* This function is called periodically during test mode
     */
    public void testPeriodic() {
        dashboard();  //Update Dashboard
    }
    
    public void dashboard() {  //Updates Dashboard with Current Values
        Watchdog watchdog = Watchdog.getInstance();
        
        SmartDashboard.putBoolean("Stop switch", (boolean)catapultStop.get());
        SmartDashboard.putNumber("Ultrasonic Reading", ultrasonic.getVoltage()*100);
        SmartDashboard.putBoolean("Hot Goal", hot);
        SmartDashboard.putBoolean("In Range", ((ultrasonic.getVoltage() * 100) < 175.0 && (ultrasonic.getVoltage() * 100) > 165.0) || ((ultrasonic.getVoltage() * 100) > 80 && (ultrasonic.getVoltage() * 100) < 100));
        
        watchdog.feed();
        
        SmartDashboard.putBoolean("Auto 1 Ball Hot Check", leftStick.getZ() <= 1.0);
        SmartDashboard.putNumber("Right Drive", rightStick.getAxis(Joystick.AxisType.kY) * 100);
        SmartDashboard.putNumber("Left Drive", leftStick.getAxis(Joystick.AxisType.kY) * 100);
        SmartDashboard.putNumber("Right Stick Y", rightStick.getAxis(Joystick.AxisType.kY));
        SmartDashboard.putNumber("Left Stick Y", leftStick.getAxis(Joystick.AxisType.kY));
        SmartDashboard.putNumber("Right Stick X", rightStick.getAxis(Joystick.AxisType.kX));
        SmartDashboard.putNumber("Left Stick X", leftStick.getAxis(Joystick.AxisType.kX));
        /*SmartDashboard.putBoolean("Auto 1 Ball No Hot Check", rightStick.getZ() <= 0.5 && rightStick.getZ() > 0.0);
        SmartDashboard.putBoolean("Auto 2 Balls", rightStick.getZ() <= -0.5);
        SmartDashboard.putBoolean("Auto No Balls No Shot", rightStick.getZ() >= 0.5);*/
        
        watchdog.feed();
    }
    
    public void setLEDColor(int color) {  //Changes Color according to input
        if(color == RED) {
            ledRed.set(Relay.Value.kReverse);
            ledGreen.set(Relay.Value.kForward);
            ledBlue.set(Relay.Value.kForward);
        } else if(color == GREEN) {
            ledRed.set(Relay.Value.kForward);
            ledGreen.set(Relay.Value.kReverse);
            ledBlue.set(Relay.Value.kForward);
        } else if(color == BLUE) {
            ledRed.set(Relay.Value.kForward);
            ledGreen.set(Relay.Value.kForward);
            ledBlue.set(Relay.Value.kReverse);
        } else if(color == YELLOW) {
            ledRed.set(Relay.Value.kReverse);
            ledGreen.set(Relay.Value.kReverse);
            ledBlue.set(Relay.Value.kForward);
        } else if(color == PURPLE) {
            ledRed.set(Relay.Value.kReverse);
            ledGreen.set(Relay.Value.kForward);
            ledBlue.set(Relay.Value.kReverse);
        } else if(color == CYAN) {
            ledRed.set(Relay.Value.kForward);
            ledGreen.set(Relay.Value.kReverse);
            ledBlue.set(Relay.Value.kReverse);
        } else if(color == WHITE) {
            ledRed.set(Relay.Value.kReverse);
            ledGreen.set(Relay.Value.kReverse);
            ledBlue.set(Relay.Value.kReverse);
        } else { // ALL OFF
            ledRed.set(Relay.Value.kForward);
            ledGreen.set(Relay.Value.kForward);
            ledBlue.set(Relay.Value.kForward);
        }
    }
    
    int ledTimer = 0;
    int randOld;
    public void setRandLED() {  //Set LED Color Randomly
        Watchdog watchdog = Watchdog.getInstance();
        ledTimer++;
        randOld = rand;
        if (ledTimer >= 40) {
            chooseLED();
            watchdog.feed();
            while(randOld == rand)
            {
                chooseLED();
                watchdog.feed();
            }
            if (rand == 0) {
                setLEDColor(RED);
            } else if (rand == 1) {
                setLEDColor(GREEN);
            } else if (rand == 2) {
                setLEDColor(BLUE);
            } else if (rand == 3) {
                setLEDColor(YELLOW);
            } else if (rand == 4) {
                setLEDColor(PURPLE);
            } else if (rand == 5) {
                setLEDColor(CYAN);
            } else if (rand == 6) {
                setLEDColor(WHITE);
            } else {
                setLEDColor(NOTHING);
            }
            
            watchdog.feed();
            ledTimer = 0;
        }
    }
    
    int rand;
    public void chooseLED() {  //Advances RNG for Random LED
        rand = randLED.nextInt(7);
    }
    
    public void setLED() {  //Sets the Color of the LED's
        DriverStation color = DriverStation.getInstance();
        Watchdog.getInstance().feed();
        
        if (catapultStop.get()) {
            if(((ultrasonic.getVoltage() * 100) < 185.0 && (ultrasonic.getVoltage() * 100) > 175.0) || ((ultrasonic.getVoltage() * 100) > 80 && (ultrasonic.getVoltage() * 100) < 100)) {
                //Turn Green if Robot is in Shooting Range
                setLEDColor(GREEN);
                Watchdog.getInstance().feed();
            } else {
                //Turn White if Catapult is Cocked
                setLEDColor(WHITE);
                Watchdog.getInstance().feed();
            }
        }
        else if (operatorController.getRawButton(2)) {
            setRandLED();  //Button 2 Randomly Sets the LEDs
            Watchdog.getInstance().feed();
        }
        else
        {
            if (color.getAlliance() == DriverStation.Alliance.kRed)
            {
                //Turn Red if Alliance Color is Red
                setLEDColor(RED);
                Watchdog.getInstance().feed();
            }
            else if (color.getAlliance() == DriverStation.Alliance.kBlue)
            {
                //Turn Blue if Alliance Color is Blue
                setLEDColor(BLUE);
                Watchdog.getInstance().feed();
            }
            else
            {
                setLEDColor(WHITE);
                Watchdog.getInstance().feed();
            }
        }
    }
    
    public boolean isHot() {  //Vision Processing for Hot Goal
        boolean result = false;
        Watchdog watchdog = Watchdog.getInstance();
        
        System.out.println("Inside isHot()");
        
        try
        {
            ColorImage image = camera.getImage();  //Get an Image
            watchdog.feed();
            BinaryImage thresholdImage = image.thresholdRGB(80, 255, 80, 255, 80, 255);  //Keep only Bright Particals
            watchdog.feed();
            BinaryImage filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles
            watchdog.feed();
            if (filteredImage.getNumberParticles() > 0)
            {
                System.out.println("PaticalCount = " + filteredImage.getNumberParticles());
                
		for (int i = 0; i < filteredImage.getNumberParticles(); i++) 
                {
                    //Analyze Particals
                    ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                    watchdog.feed();

                    if (scoreRectangularity(report) > 40)
                    {
                        //Found a Hot Goal
                        result = true;
                        break;
                    }
                }
            }
            
            //Delete Images
            filteredImage.free();
            thresholdImage.free();
            image.free();
        }
        catch(AxisCameraException ex)
        {
            System.out.println(ex.getMessage());
        }
        catch(NIVisionException ex)
        {
            System.out.println(ex.getMessage());
        }
        
        return result;
    }
    
    double scoreRectangularity(ParticleAnalysisReport report){  //Analyze Particals for Hot Goal
        if(report.boundingRectWidth*report.boundingRectHeight != 0){
            double rectRatio = (double)report.boundingRectWidth / (double)report.boundingRectHeight;
            double score = 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            System.out.println("Ratio = " + rectRatio + " Score = " + score + " Top = " + report.boundingRectTop + " Left = " + report.boundingRectLeft + " Width = " + report.boundingRectWidth + " Height = " + report.boundingRectHeight);
            if (rectRatio > 4.0 && rectRatio < 7.0)  //Similar Shape as Hot Goal
            {
                return score;
            }   
        } 
        
        return 0;
    }
}

// WE WILL WIN!
// BELIEVE!!