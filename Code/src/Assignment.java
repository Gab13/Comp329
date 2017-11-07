/*
 *  COMP329 Assignment 1
 *  Main Assignment class
 */

import java.io.DataOutputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import lejos.hardware.Battery;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Assignment {
	private static final int DIMENSION_X = 175, DIMENSION_Y = 210; // X and Y dimensions of the arena in cm
	public static final int GRID_LENGTH = 35, GRID_WIDTH = 35; // Length and width of the grid cells in cm
	
	private static int xGridCount, yGridCount;
	private static int[] cellTarget = {0, 0}, estimatedLocation = {0, 0, 0}; // Set a target cell as 0, 0 and estimated location as 0, 0, 0 (with heading)
	
	private static OccupancyGridManager gridManager;
	private static WavefrontGridManager wavefrontGridManager;

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
		xGridCount = DIMENSION_X / GRID_LENGTH;
		yGridCount = DIMENSION_Y / GRID_WIDTH;
		
		// Initialise grid manager + wavefront grid manager
		gridManager = new OccupancyGridManager(xGridCount, yGridCount);
		wavefrontGridManager = new WavefrontGridManager(xGridCount, yGridCount, gridManager);
		
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

		try {
			server = new ServerSocket(1234);
			System.out.println("Awaiting client..");
			client = server.accept();
			System.out.println("CONNECTED");
			OutputStream out = client.getOutputStream();
			dOut = new DataOutputStream(out);
		} catch(Exception e) {
			
		} 
		
		Button.waitForAnyPress();
		
		// Scan with ultrasonic sensor
		scanWithSensor();
		
		// Set target cell
		setUnexploredGridCell(estimatedLocation);

		printMessage("=== Occupancy Grid ===");
		printMessage(gridManager.printGrid());
		
		gridManager.drawMap();
		
		Button.waitForAnyPress();
		
		pilotRobot.getPilot().setLinearSpeed(10);
		pilotRobot.getPilot().setAngularSpeed(30);
		pilotRobot.getUltrasonicMotor().rotateTo(0);

		// Start arbitrator
		arbitrator.go();
		
		/*
		String row;
		int gridVal;
		
		// Check ultrasonic
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
		*/
		
		pilotRobot.endSensors();
		Button.waitForAnyPress();
		
		try {
			printMessage("Finished");
			server.close();
		} catch(Exception e) {
			
		}
	}
	
	// Print a message
	public static void printMessage(String message) {
		try {
			lcd.clear();
			System.out.println(message);
			dOut.writeUTF(message);
		} catch(Exception e) {
			
		}
	}
	
	// Does the next grid cell contain an obstacle?
	/*
	public static boolean nextGridContainsObstacle() {
		// Get current grid value based on pose
		Pose pose = opp.getPose();
		
		float x = pose.getX();
		float y = pose.getY();
		float h = pose.getHeading();

		int posX = (int) (x / GRID_LENGTH);
		int posY = (int) (y / GRID_WIDTH);
		
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
				printMessage("Obstacle in " + posX + ", " + posY + " detected!");
			} catch(Exception e) {
				
			}
			return true;
		}

		return false;
	}
	*/
	
	// Scan with ultrasonic sensor
	public static void scanWithSensor() {
		// Check canMoveForward, canRotateRight, canRotateLeft, and update occupancy grid accordingly
		Pose pose = opp.getPose();

		float h = pose.getHeading();
		
		// Get estimated cell
		int x = estimatedLocation[0], y = estimatedLocation[1];

		// Check in front
		if(h >= 45 && h < 135) {
			// Facing right from original position
			y++;
			
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			x--;
			
		} else if(h >= 225 && h < 315) {
			// Facing left
			y--;
			
		} else {
			// Facing forwards
			x++;
			
		}
		
		if(!canGoForward()) {
			gridManager.updateGridValue(x, y, 1);
		} else {
			gridManager.updateGridValue(x, y, 0);
		}
		
		x = estimatedLocation[0];
		y = estimatedLocation[1];
		
		// Check right
		if(h >= 45 && h < 135) {
			// Facing right from original position
			x--;
			
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			y--;
			
		} else if(h >= 225 && h < 315) {
			// Facing left
			x++;
			
		} else {
			// Facing forwards
			y++;
			
		}
		
		if(!canRotateRight()) {
			gridManager.updateGridValue(x, y, 1);
		} else {
			gridManager.updateGridValue(x, y, 0);
		}
		
		x = estimatedLocation[0];
		y = estimatedLocation[1];

		// Check left
		if(h >= 45 && h < 135) {
			// Facing right from original position
			x++;
			
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			y++;
			
		} else if(h >= 225 && h < 315) {
			// Facing left
			x--;
			
		} else {
			// Facing forwards
			y--;
			
		}
		
		if(!canRotateLeft()) {
			gridManager.updateGridValue(x, y, 1);
		} else {
			gridManager.updateGridValue(x, y, 0);
		}
		
		lcd.clear();
		gridManager.updateMap();
	}

	

	public static boolean canGoForward() {
		float sensorReading = pilotRobot.getUltrasonicSensor();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Is there an obstacle immediately to the right?
	public static boolean canRotateRight() {
		float sensorReading = pilotRobot.getUltrasonicSensorRight();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Is there an obstacle immediately to the left?
	public static boolean canRotateLeft() {
		float sensorReading = pilotRobot.getUltrasonicSensorLeft();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Calculate an angle given a difference
	private static double calculateAngle(float difference) {
		double ang = 0;

		ang = Math.toDegrees(Math.asin(difference/0.04)); //Distance in metres

		if ((difference/0.04) > 1) {
			ang = 360;
		}

		return ang;
	}
	
	// Aligns the robot by finding it's angle to the wall
	public static void Align(int rotate) {
		float sensorReading1 = 180; //Set it at max angle
		float sensorReading2 = 180;
		float difference;
		double angle;
		int rotation = 0;
		double offset = 2; //For dealing with error on too much rotation
		int backtrack = 4; //The distance the robot will use to measure the angle towards the wall

		pilotRobot.getPilot().travel(-backtrack);

		// If the robot has just turned right or 180 degrees

		if ((rotate == 90) || (rotate == 180)){	
			rotation = 10;

			for (int i = 0; i < 11; i++) { // Get 11 readings of he distance to the object at degrees of (rotation)
				pilotRobot.getUltrasonicMotor().rotate(rotation);

				float temp = pilotRobot.getUltrasonicSensor();

				if(temp < sensorReading1) // Get the smallest reading
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

		// If the robot has just turned left
		// Do the same as when it turns right or 180 degrees but in the opposite direction

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

		// Setting a positive length to calculate the angle of rotation
		if (difference < 0)
			difference = -difference;

		angle = calculateAngle(difference);

		// If the returned angle is 360 it means the robot has measured a much greater distance than its
		// distance to the current object and so, it backtracks further and repeats the operation
		if (angle == 360) {

			pilotRobot.getPilot().travel(-backtrack);

			Align(rotation);

		} else {
			// Adjusting the error of rotation value to 0 if the angle of rotation is too small.
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

	// Get current pose
	public static Pose getPose() {
		current = opp.getPose();
		return current; 
	}
	
	// Set a grid cell that needs exploring
	private static int[] setUnexploredGridCell(int[] currentCell) {
		int[] ret = new int[2];
		
		for(int j = 0; j < yGridCount; j++) {
			for(int i = 0; i < xGridCount; i++) {
				if(i == 0 && j == 0)
					continue;
				
				if(gridManager.getGridCounterValue(i, j) == 0 && !(i == currentCell[0] && j == currentCell[1])) {
					cellTarget[0] = i;
					cellTarget[1] = j;
					
					printMessage("Setting new target location from " + currentCell[0] + ", " + currentCell[1] + " to x:" + cellTarget[0] + ", y:" + cellTarget[1]);
					
					ret[0] = i;
					ret[1] = j;
					
					wavefrontGridManager.setPathTo(currentCell, ret);
					
					return ret;
				} else {
					if(gridManager.getGridCounterValue(i, j) == 0)
						printMessage("Skipping [" + i + "," + j + "] (1)");
					
					else if(i == currentCell[0] && j == currentCell[1])
						printMessage("Skipping [" + i + "," + j + "] (2)");
						
				}
			}
		}
		
		cellTarget[0] = 0;
		cellTarget[1] = 0;
		ret[0] = 0;
		ret[1] = 0;
		
		return ret;
	}
	
	// Get a grid cell that needs exploring
	public static int[] getUnexploredGridCell(int[] currentCell, boolean forceUpdate) {
		printMessage("Current cell = [" + currentCell[0] + "," + currentCell[1] + "]");
		
		int[] ret;
		
		if(!isMapFinished() && ((cellTarget[0] == currentCell[0] && cellTarget[1] == currentCell[1]) || forceUpdate == true)) {
			printMessage("Setting new cell...");
			ret = setUnexploredGridCell(currentCell);
		} else
			ret = cellTarget;
		
		return ret;
	}
	
	// Get the next cell the robot should travel to
	public static int[] getNextCell(int[] currentCell) {
		int[] ret = new int[2];
		
		if(!isMapFinished() && (cellTarget[0] > 0 || cellTarget[1] > 0) && !(cellTarget[0] == currentCell[0] && cellTarget[1] == currentCell[1])) {
			// Print
			printMessage("=== Path from [" + currentCell[0] + "," + currentCell[1] + "] to [" + cellTarget[0] + "," + cellTarget[1] + "] ===");
			
			int distance = wavefrontGridManager.getGridPos(currentCell[0], currentCell[1]);
			int[] nextTile;
			int[] currentTile = currentCell;
			String path = "";
			
			while(distance > 0){
				nextTile = wavefrontGridManager.getNextTile(currentTile);
				
				path += "[" + currentTile[0] + "," + currentTile[1] + "] -> [" + nextTile[0] + "," + nextTile[1] + "] ";
				
				currentTile = new int[2];
				currentTile[0] = nextTile[0];
				currentTile[1] = nextTile[1];

				distance--;
			}
			
			printMessage(wavefrontGridManager.printGrid());
			printMessage("Path from [" + currentCell[0] + "," + currentCell[1] + "] to [" + cellTarget[0] + "," + cellTarget[1] + "]: " + path);
			
			//Button.waitForAnyPress();
			
			return wavefrontGridManager.getNextTile(currentCell);
		}
		
		ret[0] = -1;
		ret[1] = -1;
		
		return ret;
	}

	// Update the robot's current location
	public static void updateLocation() {
		Pose pose = opp.getPose();
		
		float x = pose.getX();
		float y = pose.getY();
		float h = pose.getHeading();
		
		int posX = (int) (x / GRID_LENGTH);
		int posY = (int) (y / GRID_WIDTH);
		
		estimatedLocation[0] = posX;
		estimatedLocation[1] = posY;
		estimatedLocation[2] = (int) h;
		
		gridManager.updateGridValue(posX, posY, 0);
		
		printMessage("New estimated location: " + posX + ", " + posY);
		
		if(estimatedLocation[0] == cellTarget[0] && estimatedLocation[1] == cellTarget[1]) {
			getNextCell(estimatedLocation);
		}
	}
	
	// Manually set the estimated location
	public static void setLocation(int x, int y) {
		estimatedLocation[0] = x;
		estimatedLocation[1] = y;
	}
	
	// Get the robot's current estimated location
	public static int[] getEstimatedLocation() {
		printMessage("Current cell = [" + estimatedLocation[0] + "," + estimatedLocation[1] + "]");
		return estimatedLocation;
	}
	
	// Have we completed the occupancy grid?
	public static boolean isMapFinished() {
		for(int i = 0; i < xGridCount; i++) {
			for(int j = 0; j < yGridCount; j++) {
				if(gridManager.getGridCounterValue(i, j) == 0)
					return false;
			}
		}
		
		//Button.waitForAnyPress();
		lcd.clear();
		//OccupancyGridCounter.finalMap(); //G.R
		
		return true;
	}
}