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
	private static final int DIMENSION_X = 150, DIMENSION_Y = 100; // X and Y dimensions of the arena in cm
	public static final int ROBOT_LENGTH = 25, ROBOT_WIDTH = 20; // Length and width of the robot in cm
	private static int xGridCount, yGridCount;
	
	private static OccupancyGridManager gridManager;

	private static Brick myEV3;
	private static GraphicsLCD lcd;
	
	private static ServerSocket server;
	private static Socket client;
	private static DataOutputStream dOut;
	
	private static PilotRobot pilotRobot;
	private static PilotMonitor pilotMonitor;
	private static float distanceFromWall;
	private static float distanceFromObject;
	private static Pose initPose;
	private static Pose current;
	
	private static OdometryPoseProvider opp;
	
	public static void main(String[] args) {
		// Get grid size for occupancy grid + grid counter
		xGridCount = DIMENSION_Y / ROBOT_WIDTH;
		//yGridCount = DIMENSION_Y / ROBOT_WIDTH;
		yGridCount = DIMENSION_X / ROBOT_LENGTH;
		
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
		
		gridManager.drawMap(); //G.R;
		
		Button.waitForAnyPress();
		pilotRobot.getPilot().setLinearSpeed(10);
		pilotRobot.getPilot().setAngularSpeed(30);
		
		////////
		//+pilotRobot.getPilot().rotateLeft();
		//System.out.println("Rotating");
		////////
		
		
		Button.waitForAnyPress();
		
		distanceFromWall = pilotRobot.getUltrasonicSensorLeft();
		
		initPose = opp.getPose();
		pilotRobot.getUltrasonicMotor().rotateTo(0);
		
		Button.waitForAnyPress();
		// Start arbitrator
		arbitrator.go();
		
		
		/*
		String row;
		int gridVal;
		
		// Check ultrasound
		while(true) {
			Button.waitForAnyPress();
			
			if(Button.ESCAPE.isDown())
				break;
		
			float sensorReading = pilotRobot.getUltrasonicSensor();
			
			// Convert sensor reading to int
			if(sensorReading < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (sensorReading * 100);
				int gridCell = sensorReadingInt / ROBOT_LENGTH;
				//gridCell--;
				
				try {
					dOut.writeUTF("Detected obstacle at distance " + sensorReadingInt + "cm in cell (" + gridCell + ", 0)");
					dOut.flush();
				} catch(Exception e) {
					
				}
				
				if(gridCell >= 0 && gridCell < xGridCount) {
					gridManager.updateGridValue(gridCell, 0, 1);
				}
				
				lcd.clear();
				for(int i = 0; i < xGridCount; i++) {
					row = "";
					
					for(int j = 0; j < yGridCount; j++) {
						gridVal = gridManager.getGridValue(i, j);
						row += " " + gridVal;
					}
					
					lcd.drawString(row, 0, ((yGridCount - i) * 10), 0);
				}
			}
		}

		lcd.clear();
		for(int i = 0; i < xGridCount; i++) {
			row = "";
			
			for(int j = 0; j < yGridCount; j++) {
				if(gridManager.getGridCounterValue(i, j) > 0)
					gridVal = gridManager.getGridValue(i, j) / gridManager.getGridCounterValue(i, j);
				else
					gridVal = -1;
				
				row += " " + gridVal;
			}
			
			lcd.drawString(row, 0, ((yGridCount - i) * 10), 0);
		}
		
		nextGridContainsObstacle();
		
		Button.waitForAnyPress();
		
		try {
			dOut.writeUTF("Finished");
			server.close();
		} catch(Exception e) {
			
		}
		*/
	}
	
	// Print a message
	public static void printMessage(String message) {
		try {
			lcd.clear();
			dOut.writeUTF(message);
		} catch(Exception e) {
			
		}
	}
	
	// Does the next grid cell contain an obstacle?
	public static boolean nextGridContainsObstacle() {
		// Get current grid value based on pose
		Pose pose = opp.getPose();
		
		float x = pose.getX();
		float y = pose.getY();
		float h = pose.getHeading();
		
		int posX = (int) (x / ROBOT_LENGTH);
		int posY = (int) (y / ROBOT_LENGTH);
		
		if(h >= 45 && h < 135) {
			// Facing right from original position
			posY++;
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			posX--;
		} else if(h >= 225 && h < 315) {
			// Facing left
			posY--;
		} else {
			// Facing forwards
			posX++;
		}
		
		if(gridManager.getGridValue(posX,  posY) > 0) {
			try {
				dOut.writeUTF("Obstacle in " + posX + ", " + posY + " detected!");
			} catch(Exception e) {
				
			}
			return true;
		}

		return false;
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
				
				int posX = (int) (x / ROBOT_LENGTH);
				int posY = (int) (y / ROBOT_LENGTH);
				
				int sensorReadingInt = (int) (sensorReading * 100);
				int sensorReadingCell = sensorReadingInt / ROBOT_LENGTH; // Get number of cells away from robot obstacle has been detected in

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
		}
	
		

		public static boolean canGoForward() {
			distanceFromObject = pilotRobot.getUltrasonicSensor();
			
			if(distanceFromObject < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (distanceFromObject * 100);
				
				if(sensorReadingInt<= ROBOT_LENGTH)
					return false;
			}
			
			return true;
		}
	
	// Is there an obstacle immediately to the right?
	public static boolean canRotateRight() {
		float sensorReading = pilotRobot.getUltrasonicSensorRight();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= ROBOT_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Is there an obstacle immediately to the left?
	public static boolean canRotateLeft() {
		float sensorReading = pilotRobot.getUltrasonicSensorLeft();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= ROBOT_LENGTH)
				return false;
		}
		
		return true;
	}
	
	private static double calculateAngle(float difference) {
		double ang = 0;
		
		ang = Math.toDegrees(Math.asin(difference/0.03)); //Distance in metres
		return ang;
	}
	
	public static void Align(int rotate) {
		float sensorReading1 = 180;
		float sensorReading2 = 180;
		float difference;
		double angle;
		int rotation = 0;
		
		pilotRobot.getPilot().travel(-3);
		
		if (rotate == 90){	
			rotation = 10;
			for (int i = 0; i < 10; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading1)
					sensorReading1 = temp;
			}
			pilotRobot.getPilot().travel(3);
			pilotRobot.getUltrasonicMotor().rotateTo(0);
			for (int i = 0; i < 10; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading2)
					sensorReading2 = temp;
			}
		}
		else if (rotate == -90) {
			rotation = -10;
			for (int i = 0; i < 10; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading1)
					sensorReading1 = temp;
			}
			pilotRobot.getPilot().travel(3);
			pilotRobot.getUltrasonicMotor().rotateTo(0);
			
			for (int i = 0; i < 10; i++) {
				pilotRobot.getUltrasonicMotor().rotate(rotation);
				float temp = pilotRobot.getUltrasonicSensor();
				if(temp < sensorReading2)
					sensorReading2 = temp;
			}
		}
		
		pilotRobot.getPilot().stop();
		
		difference = (sensorReading1 - sensorReading2);
		
		if (difference < 0)
			difference = -difference;
		
		angle = calculateAngle(difference);
		
		System.out.println("ang " + angle);
		System.out.println("s1 " + sensorReading1);
		System.out.println("s2 " + sensorReading2);
		System.out.println("diff " + difference);
		
		if (sensorReading1 < sensorReading2)
			pilotRobot.getPilot().rotate(-angle +3);
		else
			pilotRobot.getPilot().rotate(angle -3);
		
		pilotRobot.getUltrasonicMotor().rotateTo(0);
	}
	
	public static float getWallDistance() {
		return distanceFromWall;
		}
	public static Pose getPose() {
		current = opp.getPose();
		return current; 
	}
		
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