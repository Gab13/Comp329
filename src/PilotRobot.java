
import lejos.hardware.Brick;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.SampleProvider;

public class PilotRobot {
	private Brick myEV3;
	private MovePilot pilot;
	private SampleProvider leftBumperSampleProvider, rightBumperSampleProvider, ultrasonicSensorSampleProvider, colourSensorSampleProvider;
	private float[] leftBumperSample, rightBumperSample, ultrasonicSensorSample, colourSensorSample; 
	private EV3UltrasonicSensor ultrasonicSensor;
	private EV3TouchSensor leftBumper, rightBumper;
	private EV3ColorSensor colourSensor;
	private EV3MediumRegulatedMotor ultrasonicSensorMotor;
	
	// Constructor
	public PilotRobot(Brick robot) {
		this.myEV3 = robot;
		
		// Get sensors
		this.leftBumper = new EV3TouchSensor(myEV3.getPort("S2"));
		this.rightBumper = new EV3TouchSensor(myEV3.getPort("S1"));
		this.ultrasonicSensor = new EV3UltrasonicSensor(myEV3.getPort("S3"));
		this.colourSensor = new EV3ColorSensor(myEV3.getPort("S4"));

		// Initialise sensor sample providers
		leftBumperSampleProvider = leftBumper.getTouchMode();
		rightBumperSampleProvider = rightBumper.getTouchMode();
		colourSensorSampleProvider = colourSensor.getRGBMode();
		ultrasonicSensorSampleProvider = ultrasonicSensor.getDistanceMode();
		
		// Initialise sensor samples
		leftBumperSample = new float[leftBumperSampleProvider.sampleSize()];
		rightBumperSample = new float[rightBumperSampleProvider.sampleSize()];
		colourSensorSample = new float[colourSensorSampleProvider.sampleSize()];
		ultrasonicSensorSample = new float[ultrasonicSensorSampleProvider.sampleSize()];
		
		// Initialise wheels and create chassis
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(myEV3.getPort("B"));
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(myEV3.getPort("C"));
		ultrasonicSensorMotor = new EV3MediumRegulatedMotor(myEV3.getPort("A"));
		
		Wheel leftWheel = WheeledChassis.modelWheel(leftMotor, 3.3).offset(-10.0);
		Wheel rightWheel = WheeledChassis.modelWheel(rightMotor, 3.3).offset(10.0);
		Chassis chassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		
		// Initialise move pilot with the chassis
		this.pilot = new MovePilot(chassis);
	}
	
	// Is the left bumper pressed?
	public boolean isLeftBumperPressed() {
    	leftBumperSampleProvider.fetchSample(leftBumperSample, 0);
    	return (leftBumperSample[0] == 1.0);
	}
	
	// Is the right bumper pressed?
	public boolean isRightBumperPressed() {
    	rightBumperSampleProvider.fetchSample(rightBumperSample, 0);
    	return (rightBumperSample[0] == 1.0);
	}

	public int getColour() {
		int colour = colourSensor.getColorID();
    	return colour;
	}

	// Get ultrasonic reading
	public float getUltrasonicSensor() {
		ultrasonicSensorSampleProvider.fetchSample(ultrasonicSensorSample, 0);
		return ultrasonicSensorSample[0];
	}
	
	// Get ultrasonic reading to the right
	public float getUltrasonicSensorRight() {
		// Rotate sensor by 90
		ultrasonicSensorMotor.rotate(-90);
		
		ultrasonicSensorSampleProvider.fetchSample(ultrasonicSensorSample, 0);

		float reading = ultrasonicSensorSample[0];
		// Rotate back to 0
		ultrasonicSensorMotor.rotate(90);
		
		return reading;
	}
	
	// Get ultrasonic reading to the left
	public float getUltrasonicSensorLeft() {
		// Rotate sensor by -90
		ultrasonicSensorMotor.rotate(90);
		
		ultrasonicSensorSampleProvider.fetchSample(ultrasonicSensorSample, 0);
		
		float reading = ultrasonicSensorSample[0];
		// Rotate back to 0
		ultrasonicSensorMotor.rotate(-90);
		
		return reading;
	}
	
	// Get the move pilot
	public MovePilot getPilot() {
		return pilot;
	}
	public EV3MediumRegulatedMotor getUltrasonicMotor() {
		return ultrasonicSensorMotor;
	}
	
	// End the sensors
	public void endSensors() {
		leftBumper.close();
		rightBumper.close();
		colourSensor.close();
	}
}