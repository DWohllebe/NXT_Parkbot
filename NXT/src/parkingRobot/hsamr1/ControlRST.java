package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTRegulatedMotor;
import lejos.geom.Point;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.comm.RConsole;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {


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
	 * line information measured by right light sensor.
	 */
	private int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor.
	 */
	private int lineSensorLeft	=	0;

	NXTMotor 			leftMotor 		=	null;
	NXTMotor 			rightMotor 		= 	null;
	NXTRegulatedMotor 	rightMotorR		=	null;
	NXTRegulatedMotor 	leftMotorR		=	null;

	IPerception 		perception 		= 	null;
	INavigation 		navigation 		= 	null;
	ControlThread 		ctrlThread 		= 	null;	

	Pose 				startPosition 	= 	new Pose();
	Pose 				currentPosition = 	new Pose();
	Pose 				destination 	= 	new Pose();

	ControlMode currentCTRLMODE 	= null;

	IPerception.AngleDifferenceMeasurement ADM	= null; 


	private static double KP_LL 			= 1;
	private static final double K_I 		= 0.05;		//Proportional factor for I-Element
	private static final double KD 			= 40;		//Proportional factor for D-Element
	private static final double WHEELDIAM 	= 0.056;

	private static final double TRACKWIDTH 	= 0.127;
	private static final double PI 			= 3.1415926;
	private static final double	SCOPE 		= PI*WHEELDIAM;
	

	
	private float error 		= 	0; //Difference between right and left light sensor
	private float maxerror 		= 	0; //Offset between roght and left sensor when on line
	private float last_error 	= 	0; 



	private double turnRadius 	=	0;		
	private double leftSpeed 	= 	0;			//Geschwindigkeit linkes Rad in m/s
	private double rightSpeed 	= 	0;			//Geschwindigkeit rechtes Rad in m/s
	private double nls 			=	0;			//Errechnete Solldrehzahl links
	private double nrs 			=	0;			//Errechnete Solldrehzahl rechts


	//***********SETPOSE*********************************************************************************
	private boolean arrived = false;
	//***********LINE_CTRL**********************************************************************************

	private double correctionP 	= 	0; 			//Correction power
	private double integrator 	= 	0; 			//Integrator value
	private double deriv 		= 	0;			//Derivative 
	private double turnPA 		= 	0;			//Turn Power port A
	private double turnPB 		= 	0;			//Turn Power port B

	private double velocity 	= 	0.0;
	private double x_c 			= 	0;
	private int v0 				= 	52; 		//Voreingestellte Geschwindigkeit NUR fŸr LINE_CTRL

	private double last_x_c 	= 	0;
	private double power		=	0; 			//Motorpower fŸr Linecontrol
	private double nr			=	0;			//Aktuelle Drehzahl rechts
	private double nl			= 	0; 			//Aktuelle Drehzahl links
	private double delta_t 		= 	0;			//Delta t des rechten Radencoders
	private float lastDist 		= 	0; 			//Distanz beim letzten durchlauf. Hilfsgrš§e fŸr setPose


	//***********Drive // VW_CONTROL *******************************************************************************
	private double omega 			= 	0; 			// Angualar velocity in DEG/sec
	private double error_nr 		= 	0;  		//Difference between desired and current rps. (Right wheel)
	private double error_nl 		= 	0;			//Difference between desired and current rps. (Left wheel)
	private double integratorR 		= 	0;			//I-Element for right wheel rps controller
	private double integratorL 		= 	0;			//I-Element for left wheel rps controller
	private double last_error_nr 	= 	0;			//Previous error of rps.  (Right wheel)
	private double last_error_nl 	= 	0;			//Previous error of rps.  (Left Wheel)

	//***********PARK_CTRL**********************************************************************************
	Pose curPosDeg 					= new Pose(); 	// Like current Position but heading is Degrees
	private boolean parked 			= false;
	private boolean parkedOut		= false;
	//************************************************************************************************************
	private double x_anf 			= 	0;    		// Anfangsposition fŸr x
	private double y_anf 			= 	0;    		// Anfangsposition fŸr y

	private double x_end 			= 	0;  		// Endposition fŸr x
	private double y_end 			= 	0;  		// Endposition fŸr y

	private double phi_anf 			= 	0; 			// Anfangswinkel
	private double phi_end 			= 	0; 			// Endwinkel
	//************************************************************************************************************


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

		
		this.encoderLeft  		= 	perception.getControlLeftEncoder();
		this.encoderRight 		= 	perception.getControlRightEncoder();
		this.lineSensorRight	= 	perception.getRightLineSensor();
		this.lineSensorLeft  	= 	perception.getLeftLineSensor();
		this.maxerror 			= 	perception.getMaxError();


		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}


	//Public methods
	public Pose getDest(){ return this.destination;}
	public boolean hasParked(){return this.parked;}
	public boolean parkedOut(){return this.parkedOut;}
	public boolean arrived(){return this.arrived;}
	/**
	 * Set the desired destination pose. 
	 * @param destination Desired heading must be given in DEGREES
	 * */
	public void setDestination(Pose destination){
		this.destination = destination;
	}

	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;  
	}

	/**
	 * set angular velocity
	 * @param angularVelocity in DEG/sec
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
		//this.lastTime = startTime;
	}

	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){

		switch (currentCTRLMODE)
		{
		case LINE_CTRL		: 	update_LINECTRL_Parameter();
								exec_LINECTRL_ALGO();
								break;
		case VW_CTRL		: 	update_VWCTRL_Parameter();
								exec_VWCTRL_ALGO();
								break; 
		case SETPOSE      	: 	update_SETPOSE_Parameter();
								exec_SETPOSE_ALGO();
								break;
		case PARK_CTRL		: 	update_PARKCTRL_Parameter();	  
								exec_PARKCTRL_ALGO();
								break;		  					  
		case INACTIVE 		: 	exec_INACTIVE();
								break;
		}
	}
	
	

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter()
	{
		setPose(navigation.getPose());
		angleMeasurementLeft 	= encoderLeft.getEncoderMeasurement();
		angleMeasurementRight 	= encoderRight.getEncoderMeasurement();
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter()
	{
		setPose(navigation.getPose());
		angleMeasurementLeft 	= encoderLeft.getEncoderMeasurement();
		angleMeasurementRight 	= encoderRight.getEncoderMeasurement();
		curPosDeg.setLocation(currentPosition.getLocation());
		curPosDeg.setHeading((float)(180*currentPosition.getHeading()/Math.PI));
	}

	/**
	 * update parameters during PARKING Control Mode
	 * Because {@link relativeBearing} is used. curPosDeg must be updated as well.
	 */
	private void update_PARKCTRL_Parameter(){
		setPose(navigation.getPose());
		curPosDeg.setLocation(currentPosition.getLocation());
		curPosDeg.setHeading((float)(180*currentPosition.getHeading()/Math.PI));
		angleMeasurementLeft 	= encoderLeft.getEncoderMeasurement();
		angleMeasurementRight 	= encoderRight.getEncoderMeasurement();
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in DEG/sec during VW Control Mode
	 */
	private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.omega);
	}
	
	/**
	 * This method will align the robot first. Then move towards the desired point as close as possible.
	 * Then it will turn the Robot to the desired heading 
	 **/
	private void exec_SETPOSE_ALGO()
	{	this.arrived = false;
		
		align(); //Auf Ziel ausrichten
		goStraight();		
		turnTo(destination.getHeading());
		fullStop();
		this.arrived = true;
		Button.waitForAnyPress();
	}
	
	/**
	 * Stops the engines and makes sure all parameters used for controlling the are reseted
	 * */
	private void fullStop()
	{
	rightMotor.stop();
	leftMotor.stop();
	turnPA = 0;
	turnPB = 0;
	rightMotor.setPower(0);
	leftMotor.setPower(0);
	}
	
	/**
	 * After the robot is aligned with the destination Point it will move forward until it reaches the closest 
	 * possible distance. The movement is regulated by an PD-Controller.  It does not make sense to call this method without aligning the robot first.
	 * See also {@link align()}
	 */
	private void goStraight()
	{
		double dt 		= 	0;
		double phi_c	=	curPosDeg.relativeBearing(destination.getLocation())+curPosDeg.getHeading(); //Angle for rotating the coordinate system
		x_anf 			= 	currentPosition.getX();
		y_anf 			= 	currentPosition.getY();

		lastDist = currentPosition.distanceTo(destination.getLocation());
		while(lastDist>=currentPosition.distanceTo(destination.getLocation()))
		{
			update_SETPOSE_Parameter();			
			dt = angleMeasurementRight.getDeltaT();
			if(dt!=0) //Make sure not devide by zero
			{
				deriv = (x_c-last_x_c)/dt;	
			}
			x_c=(Math.cos(Math.toRadians(phi_c))*(currentPosition.getY()-y_anf))-(Math.sin(Math.toRadians(phi_c))*(currentPosition.getX()-x_anf)); //Rotate the coordinate system

			omega = (-100*x_c-5*deriv); //PD Controller
			last_x_c = x_c;
			lastDist = currentPosition.distanceTo(destination.getLocation());
			drive(this.velocity,omega);			
			lejos.util.Delay.msDelay(100);
		}
	}
	
	/**
	 *Aligns the robot with an desired point. 
	 */
	private void align()
	{   
		boolean aligned = false;
		while (aligned == false)
		{
			update_SETPOSE_Parameter();
			RConsole.println("relBear " + curPosDeg.relativeBearing(destination.getLocation()));
			if(curPosDeg.relativeBearing(destination.getLocation()) < -1)
			{	
				if(curPosDeg.relativeBearing(destination.getLocation()) < -20)
				{
					drive(0,-30);
				}
				else
					drive(0,-20); 			
			}

			else if(curPosDeg.relativeBearing(destination.getLocation()) > 1)
			{	
				if(curPosDeg.relativeBearing(destination.getLocation()) < 20)
				{
					drive(0,20);
				}
				else drive(0,30);  				
			}

			else 
			{
				fullStop();
				aligned = true;
			}
			lejos.util.Delay.msDelay(100);
		}
	}
	
	/**
	 *Rotates the robot to an desired angle
	 *@param phi must be given in DEGREES
	 **/
	private void turnTo(double phi)
	{   
		boolean aligned = false;
		double angle = 0;
		while (aligned == false)
		{
			update_SETPOSE_Parameter();
			angle = phi - curPosDeg.getHeading();
			RConsole.println("phi - curPosDeg " + (phi - curPosDeg.getHeading()));
			if (angle < -180 )
			{
				angle += 360;
			}
			else if (angle > 180 )
			{
				angle -= 360;
			}
			
			if( angle < -1)
			{	
				if(angle < -20)
				{
					drive(0,-30);
				}
				else
					drive(0,-20); 			
			}

			else if(angle > 1)
			{	
				if(angle < 20)
				{drive(0,20);}
				else drive(0,30);  				
			}

			else 
			{
				fullStop();
				aligned = true;
			}
			lejos.util.Delay.msDelay(100);
		}
	}


	/**
	 * PARKING along the generated path. After the robot has parked this method will change the current control Mode to INACTIVE.
	 * A second change to PARK_CTRL will cause the robot to park out.
	 */
	private void exec_PARKCTRL_ALGO() {
		if (parked == false){parkIn();}	
		if(parked == true )	{parkOut();}		
	}
	
	private void parkIn()
	{
		update_PARKCTRL_Parameter();
		boolean halfWay = false;

		phi_anf = curPosDeg.getHeading();
		while (curPosDeg.getHeading() > phi_anf - 90) {
			drive(0.07, -28.64);
			update_PARKCTRL_Parameter();
			if (curPosDeg.getHeading() < phi_anf - 90) {
				halfWay = true;
				fullStop();
				break;
			}
			lejos.util.Delay.msDelay(100);
		}

		while (curPosDeg.getHeading() < phi_anf && halfWay == true) {
			drive(0.07, +28.64);
			update_PARKCTRL_Parameter();
			if (curPosDeg.getHeading() > phi_anf) {
				fullStop();
				break;
			}
			lejos.util.Delay.msDelay(100);
		}
		parked = true;
		setCtrlMode(IControl.ControlMode.INACTIVE);
		LCD.clear();
		LCD.drawString("+++Eingeparkt+++", 0, 1);
	}
	private void parkOut()
	{
		update_PARKCTRL_Parameter();
		boolean halfWay = false;

		phi_anf = curPosDeg.getHeading();
		while (curPosDeg.getHeading() < phi_anf + 90) 
		{
			drive(0.07, 28.64);
			update_PARKCTRL_Parameter();
			if (curPosDeg.getHeading() >= phi_anf + 90) {
				fullStop();
				halfWay = true;
				break;
			}
			lejos.util.Delay.msDelay(100);
		}

		while (curPosDeg.getHeading() > phi_anf && halfWay == true) 
		{
			drive(0.07, -28.64);
			update_PARKCTRL_Parameter();
			if (curPosDeg.getHeading() <= phi_anf) 
			{
				fullStop();
				break;
			}
			lejos.util.Delay.msDelay(100);
		}
		parked = false;
		setCtrlMode(IControl.ControlMode.INACTIVE);	
	
	}
	private void exec_INACTIVE(){
		this.fullStop();
	}

	/**
	 * DRIVING along black line
	 * 
	 */
	private void exec_LINECTRL_ALGO(){

		error = (lineSensorRight-lineSensorLeft);
		if (error>0 && last_error <0 || error<0 && last_error >0 ){integrator=0;} //bei VZW integrator auf 0
		integrator = integrator + error;
		ADM = encoderRight.getEncoderMeasurement();
		delta_t = ADM.getDeltaT();		
		deriv = (error-last_error)/(delta_t);
		power = v0 + (-2*v0/(maxerror))*Math.abs(error);
		if (power < 10){power = 10;}

		if(Math.abs(error) > 20) {correctionP = 4*error*KP_LL + K_I*integrator*3 + KD*deriv;}
		{correctionP = error*KP_LL + K_I*integrator + KD*deriv;}

		turnPA = power+correctionP;
		turnPB = power-correctionP;
		last_error = error;
	
		apply();
		
	}


	/**
	 *Applies the calculated motor power with respect to the correct rotation orientation.
	 *If an output power is less than zero the motor shall go backwards 
	 */
	private void apply(){
	if (turnPA < 0){							
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
	
	/**
	 * calculates the left and right angle speed of the both motors with given velocity 
	 * and angle velocity of the robot
	 * @param v velocity of the robot in m/s
	 * @param omega angle velocity of the robot in DEG/sec
	 */
	private void drive(double v, double omega){
		omega = Math.toRadians(omega);

		if (omega != 0)	
		{
			turnRadius = v/omega;
			if (turnRadius != 0)
			{
				rightSpeed= (turnRadius+(TRACKWIDTH/2f))*v/turnRadius;
				leftSpeed = (turnRadius-(TRACKWIDTH/2f))*v/turnRadius;								
				nrs = rightSpeed/SCOPE;	
				nls = leftSpeed/SCOPE;							
			}
			else{nrs = omega*TRACKWIDTH/(2f*SCOPE);
			nls = -nrs;
			}
		}
		else{nrs = nls = v/SCOPE;}
		if(angleMeasurementRight.getDeltaT()!=0 &&angleMeasurementLeft.getDeltaT()!=0 ){
			nr=(angleMeasurementRight.getAngleSum()/angleMeasurementRight.getDeltaT())*1000/360;
			nl=(angleMeasurementLeft.getAngleSum()/angleMeasurementLeft.getDeltaT())*1000/360;
		}

		error_nr = nrs-nr;
		integratorR = integratorR + error_nr;
		if (error_nr>0 && last_error_nr <0 || error_nr<0 && last_error_nr >0 ){integratorR=0;}
		last_error_nr = error_nr;
		turnPA = turnPA+20*error_nr+0*integratorR;


		error_nl = nls-nl;
		integratorL = integratorL + error_nl;
		if (error_nl>0 && last_error_nl <0 || error_nl<0 && last_error_nl >0 ){integratorL=0;}
		turnPB = turnPB+20*error_nl+0*integratorL;
		last_error_nl = error_nl;
		apply();
		
	}
	
}

