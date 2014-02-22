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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    Relay ledRed;
    Relay ledGreen;
    Relay ledBlue;
    
    DoubleSolenoid pickLift;
    DoubleSolenoid ratchetLoose;
    
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
    Servo cameraX;
    Servo cameraY;
    
    //Camera constants used for distance calculation
    final int Y_IMAGE_RES = 480;		//X Image resolution in pixels, should be 120, 240 or 480
    final double VIEW_ANGLE = 49;		//Axis M1013
    //final double VIEW_ANGLE = 41.7;		//Axis 206 camera
    //final double VIEW_ANGLE = 37.4;  //Axis M1011 camera
    final double PI = 3.141592653;

    //Score limits used for target identification
    final int  RECTANGULARITY_LIMIT = 40;
    final int ASPECT_RATIO_LIMIT = 55;

    //Score limits used for hot target determination
    final int TAPE_WIDTH_LIMIT = 50;
    final int  VERTICAL_SCORE_LIMIT = 50;
    final int LR_SCORE_LIMIT = 50;

    //Minimum area of particles to be considered
    final int AREA_MINIMUM = 150;

    //Maximum number of particles to process
    final int MAX_PARTICLES = 8;
    
    boolean hot = false;
    
    Gyro gyro;
    AnalogChannel ultrasonic;
    
    DigitalInput catapultStop;
    
    Encoder driveEncoder;
    Encoder ratchetEncoder;
    DigitalInput driveEncodeInput;
    boolean driveEncoderOld;
    boolean driveEncoderTwoOld;
    DigitalInput driveEncodeInputTwo;
    DigitalInput ratchetEncodeInput;
    DigitalInput ratchetEncodeInputTwo;
    
    Random randLED;
    Timer autoTime;
    DriverStation auto;
    
 
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
        cameraX.set(0.25);
        cameraY = new Servo(8);
        cameraY.set(0.3);
        
        pump = new Compressor(1,1);
        
        ledRed = new Relay(4);
        ledGreen = new Relay(3);
        ledBlue = new Relay(2);
        
        ratchetLoose = new DoubleSolenoid(1,4);        
        pickLift = new DoubleSolenoid(2,3); 
        
        randLED = new Random();
        randLED.nextInt(7);
        autoTime = new Timer();

        catapultStop = new DigitalInput(3);

        gyro = new Gyro(1);
        gyro.reset();
        ultrasonic = new AnalogChannel(4);
        camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        
        driveEncoder = new Encoder(6,8);
        ratchetEncoder = new Encoder(10,11);
        driveEncoder.setDistancePerPulse((2 * 3.1415 * 3.0) / 1440.0);
        driveEncoder.start();
        ratchetEncoder.start();
        //driveEncodeInput = new DigitalInput(6);
        //driveEncodeInputTwo = new DigitalInput(8);
        //ratchetEncodeInput = new DigitalInput(10);
        //ratchetEncodeInputTwo = new DigitalInput(11);
        
        
        //initializes the piston for the ratchet
        ratchetLoose.set(DoubleSolenoid.Value.kReverse);
        pickLift.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This function is called periodically during autonomous
     */
    double drive = 1.0;
    double curve = 0.03;
    
    public void autonomousInit() {
        autoTime.start();
        gyro.reset();
    }
    
    public void autonomousPeriodic() {
        dashboard();
        if (autoTime.get() == 100)
        {
             mecanum.drive(drive, curve);
             pickLift.set(DoubleSolenoid.Value.kForward);
        }
        else if (autoTime.get() == 1000)
        {
             mecanum.drive(0, 0);
        }
        else if (autoTime.get() == 3000)
        {
             ratchetLoose.set(DoubleSolenoid.Value.kForward);
        }
    System.out.println("I'm in autonomos periodic");
    }

    /**
     * This function is called periodically during operator control
     */
    double dead = 0.1;
    int pickTime = 50;

    public void teleopInit() {
         pump.start();
    }
    
    public void teleopPeriodic() {
        dashboard();
       
      
        //Calls the motor functions.
        double div;
        if(leftStick.getTrigger() == true) {
            div = 3;
        } else if (rightStick.getTrigger()){
            div = 1;
        } else div = 2;
        
        /*double leftS_X = -(leftStick.getX()/div);
        if(leftS_X > -dead &&leftS_X < dead)
        {
            leftS_X = 0;
        }
        
        double leftS_Y = leftStick.getY()/div;
        if(leftS_Y > -dead &&leftS_Y < dead)
        {
            leftS_Y = 0;
        }*/
        
        double mag = leftStick.getMagnitude()/div;
        if (mag > -dead && mag < dead)
        {
            mag = 0;
        }
        
        double deg = leftStick.getDirectionDegrees() - 90;
        
        double rightS_X = (rightStick.getX()/div);
        if(rightS_X > -dead && rightS_X < dead)
        {
            if (mag > dead && deg <= 45 && deg >= -45)
            {
                rightS_X = 0.03;
            }
            else if (mag > dead && deg > 45 && deg < 135)
            {
                rightS_X = 0.03;
            }
            else if (mag > dead && deg < -45 && deg > -135)
            {
                rightS_X = 0.03;
            }
            else if (mag > dead && deg >= 135 && deg <= -135)
            {
                rightS_X = -0.03;
            }
            else rightS_X = 0;
        }
        mecanum.mecanumDrive_Polar(mag, deg, rightS_X);
        
        operator();
        //setLED();
        ledRed.set(Relay.Value.kForward);
        ledGreen.set(Relay.Value.kForward);
        ledBlue.set(Relay.Value.kForward);
        if (leftStick.getRawButton(2))
        {
            hot = isHot();
        }
        
    }
    
    public void disabledInit() {
        
    }

    public void disabledPeriodic() {
        dashboard();
    }
    /*
     * Operator controls are interpreted here.
    */ 

    double divTwo = 1.0;
    double axis2=0;
    boolean pickDown = false;
    public void operator()
    {
        if(operatorController.getRawAxis(6) == 1.0)
        {
            pickup.set(1.0);
        }
        else if(operatorController.getRawButton(1))
        {
            pickup.set(-1.0);
        }
        else 
        {
            pickup.set(0);
        }
        
        if(operatorController.getRawButton(4))
        {
             ratchetLoose.set(DoubleSolenoid.Value.kReverse);
        }
        
        if(operatorController.getRawButton(6) && pickLift.get() == DoubleSolenoid.Value.kForward)
        {
             ratchetLoose.set(DoubleSolenoid.Value.kForward);
        }
        
        
        /*if(operatorController.getRawButton(5) == true)
        { 
            if(ratchetIn==false)
            {
                ratchetLoose.set(DoubleSolenoid.Value.kReverse);
                timer++;
                if(timer >40) ratchetIn = true;
            }else if(ratchetIn == true && catapultStop.get() == false )
            {
             ratchet.set(-0.5);
            } else ratchet.set(0);
        } else ratchet.set(0);*/
        /*
        axis2=operatorController.getRawAxis(2);
        if (axis2 > dead)
        {
            pickup.set(axis2);
        }
        else if (axis2 < -dead)
        {
            pickup.set(axis2);
        }
        else if(!operatorController.getRawButton(1) && operatorController.getRawAxis(6)>dead &&)
        {
            pickup.set(0);
        } */
        
        if (operatorController.getRawButton(5) && ! catapultStop.get())
        {
            ratchetLoose.set(DoubleSolenoid.Value.kReverse);
            ratchet.set(-1);
        }
        else
        {
            ratchet.set(0);
        }
        
        System.currentTimeMillis();
        if(operatorController.getRawButton(8))
        {
                pickLift.set(DoubleSolenoid.Value.kForward);
        }
        
        if(operatorController.getRawButton(7))
        {
                pickLift.set(DoubleSolenoid.Value.kReverse);
        }
        
        if (operatorController.getRawAxis(4) > dead)
        {
            cameraX.set(cameraX.get() + (operatorController.getRawAxis(4) / 100));
        }
        else if (operatorController.getRawAxis(4) < -dead)
        {
            cameraX.set(cameraX.get() + (operatorController.getRawAxis(4) / 100));
        }
        else
        {
            cameraX.set(cameraX.get());
        }
        
        if (operatorController.getRawAxis(5) > 0.5)
        {
            cameraY.set(cameraY.get() - (operatorController.getRawAxis(5) / 100));
        }
        else if (operatorController.getRawAxis(5) < -0.5)
        {
            cameraY.set(cameraY.get() - (operatorController.getRawAxis(5) / 100));
        }
        else
        {
            cameraY.set(cameraY.get());
        }
        
        if (operatorController.getRawButton(10))
        {
            cameraX.set(0.25);
            cameraY.set(0.3);
        }
    }
    
    
    /* This function is called periodically during test mode
     */
    public void testPeriodic() {
        dashboard();
    }
    
    public void dashboard() {
        SmartDashboard.putNumber("Speed", leftStick.getMagnitude());
        SmartDashboard.putBoolean("1/2 Power", (leftStick.getTrigger()||rightStick.getTrigger()));
        SmartDashboard.putBoolean("1/3 Power", leftStick.getTrigger());
        SmartDashboard.putBoolean("Full Power", rightStick.getTrigger());
        SmartDashboard.putNumber("X",leftStick.getX());
        SmartDashboard.putNumber("Y",leftStick.getY());
        SmartDashboard.putNumber("Turn",rightStick.getX());
        SmartDashboard.putNumber("Angle", leftStick.getDirectionDegrees());
        SmartDashboard.putBoolean("Stop switch", (boolean)catapultStop.get());
        SmartDashboard.putNumber("Ultrasonic Reading Dial", ultrasonic.getVoltage()*100);
        SmartDashboard.putNumber("Ultrasonic Reading", ultrasonic.getVoltage()*100);
        SmartDashboard.putNumber("Ultrasonic Reading Bar", ultrasonic.getVoltage()*100);
        SmartDashboard.putNumber("Gyro Reading", gyro.getAngle());
        SmartDashboard.putNumber("Driving Encoder", driveEncoder.getRaw());
        SmartDashboard.putNumber("Ratchet Encoder", ratchetEncoder.getRaw());
        //SmartDashboard.putNumber("Random Number", rand);
        //SmartDashboard.putNumber("Random Timer", ledTimer);
        SmartDashboard.putNumber("axis1",operatorController.getRawAxis(1));
        SmartDashboard.putNumber("axis2",operatorController.getRawAxis(2));
        SmartDashboard.putNumber("axis3",operatorController.getRawAxis(3));
        SmartDashboard.putNumber("axis4",operatorController.getRawAxis(4));
        SmartDashboard.putNumber("axis5",operatorController.getRawAxis(5));
        SmartDashboard.putNumber("axis6",operatorController.getRawAxis(6));
        SmartDashboard.putBoolean("Ratchet engaged", ratchetLoose.get() == DoubleSolenoid.Value.kForward);
        SmartDashboard.putBoolean("Hot Goal?", hot);
        
        //SmartDashboard.putBoolean("Driving Encoder", (boolean)driveEncodeInput.get());
        //SmartDashboard.putBoolean("Driving EncoderTwo", (boolean)driveEncodeInputTwo.get());
        //if (driveEncoderOld != driveEncodeInput.get())
        //{
        //    System.out.println("Driving Encoder " + driveEncodeInput.get());
        //    driveEncoderOld = driveEncodeInput.get();
        //}
        //if (driveEncoderTwoOld != driveEncodeInputTwo.get())
        //{
        //    System.out.println("Driving Encoder Two" + driveEncodeInputTwo.get());
        //    driveEncoderTwoOld = driveEncodeInputTwo.get();
        //}
    }
    
    public void setLEDColor(int color) {
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
    public void setLED() {
        ledTimer++;
        randOld = rand;
        if (ledTimer >= 50) {
            chooseLED();
            while(randOld == rand)
            {
                chooseLED();
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
            
            ledTimer = 0;
        }
    }
    
    int rand;
    public void chooseLED() {
        rand = randLED.nextInt(7);
    }
    
    public boolean isHot() {
        boolean result = false;
        Watchdog watchdog = Watchdog.getInstance();
        
        System.out.println("Inside isHot()");
        
        try
        {
            ColorImage image = camera.getImage();
            watchdog.feed();
            //BinaryImage thresholdImage = image.thresholdHSV(105, 137, 230, 255, 133, 183);   // keep only green objects
            BinaryImage thresholdImage = image.thresholdRGB(0, 128, 50, 255, 0, 128);
            watchdog.feed();
            BinaryImage filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles
            watchdog.feed();
            if (filteredImage.getNumberParticles() > 0)
            {
                System.out.println("PaticalCount=" + filteredImage.getNumberParticles());
                
		for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) 
                {
                    ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                    watchdog.feed();

                    System.out.println("Score=" + scoreRectangularity(report));
                    if (scoreRectangularity(report) > 70)
                    {
                        result = true;
                        break;
                    }
                }
            }
            
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
    
    double scoreRectangularity(ParticleAnalysisReport report){
        if(report.boundingRectWidth*report.boundingRectHeight !=0){
                return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
        } else {
                return 0;
        }	
    }
}