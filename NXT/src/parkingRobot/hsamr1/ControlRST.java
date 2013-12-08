package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	//private
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	ControlThread ctrlThread = null;	

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null; 
	EncoderSensor controlLeftEncoder     = null;
	IPerception.AngleDifferenceMeasurement ADM		= null; 
	int lastTime = 0;
	double lineSensorRightValue = 0;
	double lineSensorLeftValue = 0;
	float error = 0; //Differecne between right and left light sensor
	int e_ll_old = 0;	//Previous error value
	int delta_e = 0;	//Current error minus old error
	long delta_t = 0;	//Time difference
	float maxerror = 0; //Offset between roght and left sensor when on line
	float last_error = 0; 
	int count = 0;
	static final double WHEELDIAM = 5.6;
	static final double TRACKWIDTH = 13.5;
	static final double	SCOPE = 3.1415926*WHEELDIAM;
	static final double distPerDeg = SCOPE/360;
	double omega = 0;
	double turnRadius =	0;
	double leftSpeed = 0;
	double rightSpeed = 0;
	double nl =0;
	double nr =0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    static final double KP_LL1 = 0.2; 		//Proportional factor for P-Element (lego sensor)
    static final double KP_LL2 = 1;
    static double KP_LL = 2;
    static final double K_I = 0.04;		//Proportional factor for I-Element
    static final double KD = 20;			//Proportional factor for D-Element
    double offset_ll = 0; 	//Offset of light sensor for P-Element (lego sensor)
    double offset_lr = 0;	//Offset for right light sensor
    double e_ll = 0;	//Error for lego light sensor
    double correctionP = 0; //Correction power
	double integrator = 0; 	//Integrator value
	double deriv = 0;		//Derivative 
	double turnPA = 0;		//Turn Power port A
	double turnPB = 0;		//Turn Power port B
	double last_e_ll = 0;
	double velocity = 15;
	double derivm0 = 0;
	double derivm1 = 0;
	double derivm2 = 0;
	double derivm3 = 0;
	static final double DERIVMIN = 0.075;
    double last_deriv = 0;
	 
    //Beste einstellung: KP_LL 0.6; KI 0.08; KD 0.8; Geschw.: 29 Abstand: sehr weit; Runden:2
	
	//Robust geradeaus hšhe ca. 3 KP_LL 0.53; KI 0.035; KD 50; Geschw.: 33 Abstand: 2.5; Runden:0
	//Beste Einstellung hoch: hšhe ca. 2 KP_LL 2.3; KI 0.032; KD 6; Geschw.: 33 Abstand: 4; Runden: >10
    
    //Beste einstellung schnell: hšhe ca. 2.5 KP_LL 2.1; KI 0.039; KD 20; Geschw.: 50 Abstand: 4; Runden: 7
    
    //KP_LL 0.5; KI 0.07; KD 0.5; Geschw.: 25 Abstand: weit; Runden:1,5
	//KP_LL 0.5; KI 0.07; KD 0.0; Geschw.: 25 Abstand: weit; Runden:2
	//KP_LL 0.5; KI 0.07; KD 2; Geschw.: 25 Abstand: weit; Runden:0
	//KP_LL 0.5; KI 0.07; KD 5; Geschw.: 25 Abstand: weit; Runden:0
	//KP_LL 0.5; KI 0.07; KD 15; Geschw.: 25 Abstand: weit; Runden:0
	//KP_LL 0.5; KI 0.07; KD 15; Geschw.: 25 Abstand: mittel; Runden:0
	



	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor){
		this.perception = perception;
        this.navigation = navigation;
		this.leftMotor 	= leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
				
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;  //Velocity ranges from 0 to 700
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.omega = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	
	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		//this.lineSensorRightValue	= perception.getRightLineSensorValue();
		//this.lineSensorLeftValue	= perception.getLeftLineSensorValue();
		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.omega);
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		
		error = (lineSensorRight-lineSensorLeft);
		if (error>0 && last_error <0 || error<0 && last_error >0 ){integrator=0;} //bei VZW integrator auf 0
		integrator = integrator + error;
		ADM = encoderRight.getEncoderMeasurement();
		delta_t = ADM.getDeltaT();		
		deriv = (error-last_error)/(delta_t);
		
		if(Math.abs(error)>3){
		velocity = velocity/(1.3*Math.log(Math.abs(error)));}
		//velocity = velocity-2*Math.abs(error); 
		
		//velocity = velocity - 3*Math.abs(error);
		if (deriv > derivm0){derivm0=deriv;}
		else if (deriv > derivm1){derivm1=deriv;}
		else if (deriv > derivm2){derivm2=deriv;}
		else if (deriv > derivm3){derivm3=deriv;}
		/*if (Math.abs(deriv) > DERIVMIN && Math.abs(deriv) > Math.abs(last_deriv))
			{
			count =40;
			last_deriv = deriv;
			}*/
		//if (count > 0)
		if (Math.abs(deriv) > DERIVMIN && Math.abs(deriv) > Math.abs(last_deriv) && Math.abs(error)>=1)
			{correctionP = error * (KP_LL+0.35*Math.log((Math.abs(error))) /*0.055*error*/) + K_I*integrator + KD*deriv*4;}
		else correctionP = error*KP_LL + K_I*integrator + KD*deriv;
		if(count==0){last_deriv=0;}
		turnPA = velocity+correctionP;
		turnPB = velocity-correctionP;
		last_error = error;
		velocity = 50;
		/*
		error = (lineSensorRight-lineSensorLeft);
		if (error>0 && last_error <0 || error<0 && last_error >0 ){integrator=0;} //bei VZW integrator auf 0		
		integrator = integrator + error;		
		ADM = encoderRight.getEncoderMeasurement();
		delta_t = ADM.getDeltaT();		
		deriv = (error-last_error)/(delta_t);
		//if (error<5 && error>-5){deriv=0;}
		//if(error<7 && error>-7){KP_LL = KP_LL1;}
		//else {KP_LL=KP_LL2;}
		if(error<0){
			correctionP = Math.pow(10,(-error/maxerror)-1) * KP_LL*error + K_I*integrator + KD*deriv;
			}
		else{correctionP = Math.pow(10,(error/maxerror)-1) * KP_LL*error + K_I*integrator + KD*deriv;}
		
		turnPA = velocity+correctionP;
		turnPB = velocity-correctionP;
		last_error = error;
		*/
		LCD.clear();	
		LCD.drawString("SensorR: " + lineSensorRight , 0, 0);
		LCD.drawString("SensorL: " + lineSensorLeft , 0, 1);
		LCD.drawString("error: " + error , 0, 2);
		LCD.drawString("Velocity: " + velocity , 0, 3);
		LCD.drawString("TPA: " + turnPA, 0, 4);
		LCD.drawString("TPB: " + turnPB, 0, 5);
		LCD.drawString("CP: " + correctionP, 0, 6);
		if (turnPA < 0){							//if an output power is less than zero the motor shall go backwards
			//turnPA = -turnPA;
			rightMotor.setPower((int)(-turnPA));
			leftMotor.setPower((int)turnPB);
			rightMotor.backward();			
			leftMotor.forward();
		}
		
		else if(turnPB < 0){
			//turnPB = -turnPB;
			leftMotor.setPower((int)(-turnPB));
			rightMotor.setPower((int)turnPA);
			leftMotor.backward();
			rightMotor.forward();
			
			}
			
		else{
			rightMotor.setPower((int)(turnPA));
			leftMotor.setPower((int)(turnPB));		
			rightMotor.forward();
			leftMotor.forward();
			}
		
	}
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
		}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(double v, double omega){
		
		angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		angleMeasurementRight = encoderRight.getEncoderMeasurement();
		if (omega != 0)	
		{
			turnRadius = v/omega;
			if (turnRadius != 0)
				{
				rightSpeed= (turnRadius+(TRACKWIDTH/2))*v/turnRadius;
				leftSpeed = (turnRadius-(TRACKWIDTH/2))*v/turnRadius;								
				nr = rightSpeed/SCOPE;	
				nl = leftSpeed/SCOPE;							
				}
			else{nr = omega*TRACKWIDTH/(2*SCOPE);
				nl = -nr;
				}
		}
		else{nr = nl = v/SCOPE;}
		
		if((angleMeasurementRight.getAngleSum()/angleMeasurementRight.getDeltaT())*1000/360<nr){turnPA++;}
		if((angleMeasurementRight.getAngleSum()/angleMeasurementRight.getDeltaT())*1000/360>nr){turnPA--;}
		
		if((angleMeasurementLeft.getAngleSum()/angleMeasurementLeft.getDeltaT())*1000/360<nl){turnPB++;}
		if((angleMeasurementLeft.getAngleSum()/angleMeasurementLeft.getDeltaT())*1000/360>nl){turnPB--;}
		
		LCD.clear();
		LCD.drawString("nls: " + nl, 0, 1);
		LCD.drawString("nrs: "+nr , 0, 2);
		LCD.drawString("nr: " + (angleMeasurementRight.getAngleSum()/angleMeasurementRight.getDeltaT())*1000/360 , 0, 3);
		LCD.drawString("nl: " + (angleMeasurementLeft.getAngleSum()/angleMeasurementLeft.getDeltaT())*1000/360 , 0, 4);
		LCD.drawString("TPB: " + turnPB, 0, 5);
		LCD.drawString("TPA: " + turnPA, 0, 6);
		
		if (turnPA < 0){							//if an output power is less than zero the motor shall go backwards
			rightMotor.setPower((int)(-turnPA));
			leftMotor.setPower((int)turnPB);
			rightMotor.backward();			
			leftMotor.forward();
		}
		
		else if(turnPB < 0){
			leftMotor.setPower((int)(-turnPB));
			rightMotor.setPower((int)turnPA);
			leftMotor.backward();
			rightMotor.forward();
			
			}
			
		else{
			rightMotor.setPower((int)(turnPA));
			leftMotor.setPower((int)(turnPB));		
			rightMotor.forward();
			leftMotor.forward();
			}
		}
		
	}

