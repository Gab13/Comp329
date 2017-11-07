/*
 *  COMP329 Assignment 1
 *  Main Assignment class
 */

import java.io.DataOutputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Battery;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import java.lang.Math;

public class Assignment {
	private static final int DIMENSION_X = 175, DIMENSION_Y = 210; // X and Y dimensions of the arena in cm
	public static final int TILE_LENGTH = 35, TILE_WIDTH = 35; // Length and width of the robot in cm
	private static int xGridCount, yGridCount;
	
	private static OccupancyGridManager gridManager;

	private static Brick myEV3;
	private static GraphicsLCD lcd;
	
	private static ServerSocket server;
	private static Socket client;
	private static DataOutputStream dOut;
	
	private static PilotRobot pilotRobot;
	private static PilotMonitor pilotMonitor;
	private static float distanceFromObject;
	private static Pose current;
	
	private static OdometryPoseProvider opp;
	
	public static void main(String[] args) {
		// Get grid size for occupancy grid + grid counter
		xGridCount = DIMENSION_X / TILE_LENGTH; //To include walls
		//yGridCount = DIMENSION_Y / ROBOT_WIDTH;
		yGridCount = DIMENSION_Y / TILE_WIDTH;
		
		// Initialise grid manager
		gridManager = new OccupancyGridManager(xGridCount, yGridCount);
		
		// Set current cell as unoccupied as the robot is in it
		gridManager.updateGridValue(0, 0, 0);
		
		// Initialise robot objects
		myEV3 = BrickFinder.getDefault();
		lcd = myEV3.getGraphicsLCD();
		pilotRobot = new PilotRobot(myEV3);
		pilotMonitor = new PilotMonitor(pilotRobot, 400);
		opp = new OdometryPoseProvider(pilotRobot.getPilot());
		
		// Create behaviours and an arbitrator
		Behavior navigateMap = new BehaviourNavigateMap(pilotRobot);
		Behavior navigateObstacle = new BehaviourNavigateObstacle(pilotRobot);
		//Behavior bumpersPressed = new BehaviourBumpersPressed(pilotRobot);
		
		Behavior[] behaviours = {navigateMap, navigateObstacle};
		
		Arbitrator arbitrator = new Arbitrator(behaviours);

		/*
		try {
			server = new ServerSocket(1234);
			System.out.println("Awaiting client..");
			client = server.accept();
			System.out.println("CONNECTED");
			OutputStream out = client.getOutputStream();
			dOut = new DataOutputStream(out);
		} catch(Exception e) {
			
		} */
		
		//G.R//
		gridManager.drawMap();
		
		pilotRobot.getPilot().setLinearSpeed(10);
		pilotRobot.getPilot().setAngularSpeed(30);		
		
		pilotRobot.getUltrasonicMotor().rotateTo(0);
		//G.R//
		
		Button.waitForAnyPress();
		// Start arbitrator
		arbitrator.go();
		
	}
	
	// Print a message
	public static void printMessage(String message) {
		try {
			lcd.clear();
			dOut.writeUTF(message);
		} catch(Exception e) {
			
		}
	}
	
	// Scan with ultrasound sensor
		public static void scanWithSensor() {
			float sensorReading = pilotRobot.getUltrasonicSensor();
			String row;
			int gridVal;

			// Convert sensor reading to int
			if(sensorReading < Float.POSITIVE_INFINITY) {
				Pose pose = opp.getPose();
				
				float x = pose.getX();
				float y = pose.getY();
				float h = pose.getHeading();
				
				int posX = (int) (x / TILE_LENGTH);
				int posY = (int) (y / TILE_WIDTH);
				
				int sensorReadingInt = (int) (sensorReading * 100);
				int sensorReadingCell = sensorReadingInt / TILE_LENGTH; // Get number of cells away from robot obstacle has been detected in

				if(h >= 45 && h < 135) {
					// Facing right from original position
					posY += sensorReadingCell;
				} else if(h >= 135 && h < 225) {
					// Facing back towards original position
					posX -= sensorReadingCell;
				} else if(h >= 225 && h < 315) {
					// Facing left
					posY -= sensorReadingCell;
				} else {
					// Facing forwards
					posX += sensorReadingCell;
				}

				if(posX >= 0 && posY >= 0 && posX < xGridCount && posY < yGridCount) {
					gridManager.updateGridValue(posX, posY, 1);
					
				} //System.out.println(posX +""+ posY);
				
				lcd.clear();
				for(int i = 0; i < xGridCount; i++) {
					row = "";
					
					for(int j = 0; j < yGridCount; j++) {
						gridVal = gridManager.getGridValue(i, j);
						row += " " + gridVal;
					}
					lcd.clear();
					//lcd.drawString(row, 0, ((yGridCount - i) * 10), 0);
				}
			} lcd.clear();
			gridManager.updateMap();
			//System.out.println("");
			//gridManager.printgrid();
		}
	
		

		public static boolean canGoForward() {
			distanceFromObject = pilotRobot.getUltrasonicSensor();
			
			if(distanceFromObject < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (distanceFromObject * 100);
				
				if(sensorReadingInt<= TILE_LENGTH)
					return false;
			}
			
			return true;
		}
	
	// Is there an obstacle immediately to the right?
	public static boolean canRotateRight() {
		float sensorReading = pilotRobot.getUltrasonicSensorRight();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= TILE_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Is there an obstacle immediately to the left?
	public static boolean canRotateLeft() {
		float sensorReading = pilotRobot.getUltrasonicSensorLeft();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= TILE_LENGTH)
				return false;
		}
		
		return true;
	}
	
	private static double calculateAngle(float difference) {
		double ang = 0;
		
		ang = Math.toDegrees(Math.asin(difference/0.04)); //Distance in metres
		
		if ((difference/0.04) > 1) {
			ang = 360;
		}
		
		return ang;
	}
	
	//G.R//
	//Aligns the robot by finding it's angle to the wall
	public static void Align(int rotate) {
		float sensorReading1 = 180; //Set it at max angle
		float sensorReading2 = 180;
		float difference;
		double angle;
		int rotation = 0;
		double offset = 2; //For dealing with error on too much rotation
		int backtrack = 4; //The distance the robot will use to measure the angle towards the wall
		
		pilotRobot.getPilot().travel(-backtrack);
		
		//If the robot has just turned right or 180 degrees
		if ((rotate == 90) || (rotate == 180)){	
			rotation = 10;
			for (int i = 0; i < 11; i++) { //Get 11 readings of he distance to the object at degrees of (rotation)
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading1) //Get the smallest reading
					sensorReading1 = temp;
			}
			pilotRobot.getPilot().travel(backtrack); //Repeat to get shortest second reading
			pilotRobot.getUltrasonicMotor().rotateTo(0);
			for (int i = 0; i < 11; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading2)
					sensorReading2 = temp;
			}
		}
		//If the robot has just turned left
		//Do the same as when it turns right or 180 degrees but in the opposite direction
		else if (rotate == -90) {
			rotation = -10;
			for (int i = 0; i < 11; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading1)
					sensorReading1 = temp;
			}
			pilotRobot.getPilot().travel(backtrack);
			pilotRobot.getUltrasonicMotor().rotateTo(0);
			
			for (int i = 0; i < 11; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading2)
					sensorReading2 = temp;
			}
		}
		
		pilotRobot.getPilot().stop();
		
		difference = (sensorReading1 - sensorReading2);
		
		//Setting a positive length to calculate the angle of rotation
		if (difference < 0)
			difference = -difference;
		
		angle = calculateAngle(difference);
		
		//If the returned angle is 360 it means the robot has measured a much greater distance than its
		//distance to the current object and so, it backtracks further and repeats the operation
		if (angle == 360) {
			pilotRobot.getPilot().travel(-backtrack);
			Align(rotation);
		}
		else {
		
			/*
		System.out.println("ang " + angle);
		System.out.println("s1 " + sensorReading1);
		System.out.println("s2 " + sensorReading2);
		System.out.println("diff " + difference);
		*/
		
		//Adjusting the error of rotation value to 0 if the angle of rotation is too small.
		if ((angle > 0) && (angle <= 5)) {
			offset = 0;
			angle = 0;
		}
		else offset = 2;
		
		
		if (angle <= 20)
			offset = 0;
		
		if ((rotate == 90) || (rotate == 180)){
			if (sensorReading1 < sensorReading2)
				pilotRobot.getPilot().rotate(-angle -offset);
			else
				pilotRobot.getPilot().rotate(angle + offset);
		}
		if (rotate == -90){
			if (sensorReading1 < sensorReading2)
				pilotRobot.getPilot().rotate(angle + offset);
			else
				pilotRobot.getPilot().rotate(-angle -offset);
		}
		
		
		pilotRobot.getUltrasonicMotor().rotateTo(0);
		}
	}
	public static Pose getPose() {
		current = opp.getPose();
		return current; 
	}
	//G.R//
	
	// Have we completed the occupancy grid?
	public static boolean isMapFinished() {
		lcd.clear();
		for(int i = 0; i < xGridCount; i++) {
			for(int j = 0; j < yGridCount; j++) {
				printMessage("Grid " + i + ", " + j + " => " + gridManager.getGridCounterValue(i, j));
				if(gridManager.getGridCounterValue(i, j) == 0)
					return false;
			}
		}
		
		Button.waitForAnyPress();
		lcd.clear();
		//OccupancyGridCounter.finalMap(); //G.R
		
		return true;
	}
}