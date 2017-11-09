/*
 *  COMP329 Assignment 1
 *  Main Assignment class
 */

import java.io.DataOutputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Assignment {
	private static final int DIMENSION_X = 192, DIMENSION_Y = 160; // X and Y dimensions of the arena in cm
	public static final int GRID_LENGTH = 32, GRID_WIDTH = 32; // Length and width of the grid cells in cm
	public static boolean canGoToFinalCell = false; // If we've finished exploring the map, the robot can go to the final cell
	
	private static int xGridCount, yGridCount;
	private static int[] cellTarget = {0, 0}, finalCell = {0, 0}, estimatedLocation = {0, 0, 0}; // Set a target and final cell as 0, 0 and estimated location as 0, 0, 0 (with heading)
	
	private static OccupancyGridManager gridManager;
	private static WavefrontGridManager wavefrontGridManager;

	private static Brick myEV3;
	private static GraphicsLCD lcd;
	
	private static ServerSocket server;
	private static Socket client;
	private static DataOutputStream dOut;
	
	private static PilotRobot pilotRobot;

	private static float distanceFromObject;
	private static Pose current;
	
	private static Arbitrator arbitrator;
	
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
		opp = new OdometryPoseProvider(pilotRobot.getPilot());

		// Create behaviours and an arbitrator
		Behavior navigateMap = new BehaviourNavigateMap(pilotRobot);
		Behavior[] behaviours = {navigateMap};
		
		arbitrator = new Arbitrator(behaviours);

		// Connect with PC via bluetooth
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

		for(int i = 0; i < 15; i++) {
			System.out.println("");
		}
		
		gridManager.drawMap();
		lcd.clear();
		
		// Linear speed 15 for tracked robot, 20 for 2 wheeled
		pilotRobot.getPilot().setLinearSpeed(20);
		pilotRobot.getPilot().setAngularSpeed(30);
		pilotRobot.getUltrasonicMotor().rotateTo(0);

		// Start arbitrator
		arbitrator.go();
	}
	
	// Print a message
	public static void printMessage(String message) {
		try {
			dOut.writeUTF(message);
		} catch(Exception e) {
			
		}
	}
	
	// Scan with ultrasonic sensor
	public static void scanWithSensor() {
		// Check canMoveForward, canRotateRight, canRotateLeft, and update occupancy grid accordingly
		canGoForward();
		canRotateRight();
		canRotateLeft();
		
		lcd.clear();
		gridManager.updateMap();
	}

	// Is there an obstacle in front?
	public static boolean ObstacleInFront() {
		distanceFromObject = pilotRobot.getUltrasonicSensor();
		
		if(distanceFromObject < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (distanceFromObject * 100);
			
			if(sensorReadingInt<= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Check if the robot can go forward using the occupancy grid
	public static boolean canGoForward() {
		float sensorReading;
		int x, y, h;

		// Scan 5 times
		for(int i = 0; i < 5; i++) {
			x = estimatedLocation[0];
			y = estimatedLocation[1];
			h = estimatedLocation[2];
			
			sensorReading = pilotRobot.getUltrasonicSensor();
			
			if(sensorReading < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (sensorReading * 100);

				if(sensorReadingInt <= 25) {
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

					gridManager.updateGridValue(x, y, 1);
					
					lcd.clear();
					gridManager.updateMap();
					
				} else {
					gridManager.updateGridValue(x, y, 0);
				}
			} else {
				gridManager.updateGridValue(x, y, 0);
			}
		}
		
		x = estimatedLocation[0];
		y = estimatedLocation[1];
		h = estimatedLocation[2];
		
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

		if(gridManager.getProbability(x, y) > 0.5)
			return false;
		
		return true;
	}
	
	// Check if the robot can go right using the occupancy grid
	public static boolean canRotateRight() {
		float sensorReading;
		int x, y, h;
		
		pilotRobot.rotateUltrasonicSensor(-90);
		
		// Scan 5 times
		for(int i = 0; i < 5; i++) {
			x = estimatedLocation[0];
			y = estimatedLocation[1];
			h = estimatedLocation[2];
			
			sensorReading = pilotRobot.getUltrasonicSensor();
			
			if(sensorReading < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (sensorReading * 100);
				
				if(sensorReadingInt <= 25) {
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

					gridManager.updateGridValue(x, y, 1);
					
					lcd.clear();
					gridManager.updateMap();
					
				} else {
					gridManager.updateGridValue(x, y, 0);
				}
			} else {
				gridManager.updateGridValue(x, y, 0);
			}
		}
		
		pilotRobot.rotateUltrasonicSensor(90);
		
		x = estimatedLocation[0];
		y = estimatedLocation[1];
		h = estimatedLocation[2];
		
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
		
		if(gridManager.getProbability(x, y) > 0.5)
			return false;
		
		return true;
	}
	
	// Check if the robot can go left using the occupancy grid
	public static boolean canRotateLeft() {
		float sensorReading;
		int x, y, h;
		
		pilotRobot.rotateUltrasonicSensor(90);
		
		// Scan 5 times
		for(int i = 0; i < 5; i++) {
			x = estimatedLocation[0];
			y = estimatedLocation[1];
			h = estimatedLocation[2];
			
			sensorReading = pilotRobot.getUltrasonicSensor();
			
			if(sensorReading < Float.POSITIVE_INFINITY) {
				int sensorReadingInt = (int) (sensorReading * 100);
				
				if(sensorReadingInt <= 25) {
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

					gridManager.updateGridValue(x, y, 1);
					
					lcd.clear();
					gridManager.updateMap();
					
				} else {
					gridManager.updateGridValue(x, y, 0);
				}
			} else {
				gridManager.updateGridValue(x, y, 0);
			}
		}
		
		pilotRobot.rotateUltrasonicSensor(-90);
		
		x = estimatedLocation[0];
		y = estimatedLocation[1];
		h = estimatedLocation[2];
		
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
		
		if(gridManager.getProbability(x, y) > 0.5)
			return false;
		
		return true;
	}
	
	// Calculate an angle given a difference
	/*
	 *  Unused function - was used by the alignment function
	 *
	private static double calculateAngle(float difference) {
		double ang = 0;

		ang = Math.toDegrees(Math.asin(difference/0.04)); //Distance in metres

		if ((difference/0.04) > 1) {
			ang = 360;
		}

		return ang;
	}
	*/
	
	///-------------------------
	

	// Is there an obstacle immediately to the right?
	/*
	 *  Unused functions - were used by the alignment function
	 * 
	public static boolean canGoRight() {
		float sensorReading = pilotRobot.getUltrasonicSensorRight();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	
	// Is there an obstacle immediately to the left?
	public static boolean canGoLeft() {
		float sensorReading = pilotRobot.getUltrasonicSensorLeft();
		
		if(sensorReading < Float.POSITIVE_INFINITY) {
			int sensorReadingInt = (int) (sensorReading * 100);
			
			if(sensorReadingInt <= GRID_LENGTH)
				return false;
		}
		
		return true;
	}
	*/
	
	// Aligns the robot by finding it's angle to the wall
	/*
	 *  Unused function - was used to align robot with the use of an obstacle
	 * 
	public static void Align() {
		float sensorReading1 = 180; //Set it at max angle
		float sensorReading2 = 180;
		float difference;
		double angle;
		int rotation = 0;
		double offset = 3; //For dealing with error on too much rotation
		int backtrack = 4; //The distance the robot will use to measure the angle towards the wall

		pilotRobot.getUltrasonicMotor().rotateTo(0);
		pilotRobot.getPilot().travel(-backtrack);

		// If the robot has just turned right or 180 degrees

		if (!canGoLeft()){	
			rotation = 10;

			pilotRobot.getUltrasonicMotor().rotateTo(0);
			
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

		else if (!canGoRight()) {
			rotation = -10;
			
			pilotRobot.getUltrasonicMotor().rotateTo(0);

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

			Align();
		} else {
			// Adjusting the error of rotation value to 0 if the angle of rotation is too small.
			if ((angle > 0) && (angle <= 5)) {
				offset = 0;
				angle = 0;
			}
	
			else offset = 3;
	
			if (angle <= 20)
				offset = 0;
	
			if (!canGoLeft()){
				pilotRobot.getUltrasonicMotor().rotateTo(0);
				if (sensorReading1 < sensorReading2)
					pilotRobot.getPilot().rotate(-angle -offset);
				else
					pilotRobot.getPilot().rotate(angle + offset);
			}
			
			if (!canGoRight()){
				pilotRobot.getUltrasonicMotor().rotateTo(0);
				if (sensorReading1 < sensorReading2)
					pilotRobot.getPilot().rotate(angle + offset);
				else
					pilotRobot.getPilot().rotate(-angle -offset);
			}
	
			pilotRobot.getUltrasonicMotor().rotateTo(0);

		}
		pilotRobot.getUltrasonicMotor().rotateTo(0);
	}
	*/

	// Get current pose
	public static Pose getPose() {
		current = opp.getPose();
		return current; 
	}
	
	// Set a grid cell that needs exploring
	private static int[] setUnexploredGridCell(int[] currentCell) {
		int[] ret = new int[2];
		
		if(canGoToFinalCell) {
			wavefrontGridManager.setPathTo(currentCell, finalCell);
			
			return finalCell;
			
		} else {
			for(int i = (yGridCount - 1); i >= 0; i--) {
				for(int j = (xGridCount - 1); j >= 0; j--) {
					if(i == 0 && j == 0)
						continue;
	
					if(gridManager.getGridCounterValue(j, i) == 0 && !(j == currentCell[0] && i == currentCell[1])) {
						cellTarget[0] = j;
						cellTarget[1] = i;

						ret[0] = j;
						ret[1] = i;
						
						wavefrontGridManager.setPathTo(currentCell, ret);
						
						return ret;
					}
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
		int[] ret;
		
		if(!isMapFinished() && ((cellTarget[0] == currentCell[0] && cellTarget[1] == currentCell[1]) || forceUpdate == true)) {
			ret = setUnexploredGridCell(currentCell);
		} else {
			if(canGoToFinalCell)
				ret = setUnexploredGridCell(currentCell);
			else
				ret = cellTarget;
		}

		return ret;
	}
	
	// Get the next cell the robot should travel to
	public static int[] getNextCell(int[] currentCell) {
		int[] ret = new int[2];
		
		// Get the next cell if the map is not finished yet, and we are still exploring, get the next cell from wavefront search 
		if(!isMapFinished() && (cellTarget[0] > 0 || cellTarget[1] > 0) && !(cellTarget[0] == currentCell[0] && cellTarget[1] == currentCell[1])) {
			// Print
			int distance = wavefrontGridManager.getGridPos(currentCell[0], currentCell[1]);
			int[] nextTile;
			int[] currentTile = currentCell;

			while(distance > 0){
				nextTile = wavefrontGridManager.getNextTile(currentTile);

				currentTile = new int[2];
				currentTile[0] = nextTile[0];
				currentTile[1] = nextTile[1];

				distance--;
			}

			return wavefrontGridManager.getNextTile(currentCell);
			
		// Otherwise, if we are going to the final cell, get the next cell from wavefront search
		} else if(canGoToFinalCell) {
			if(currentCell[0] == finalCell[0] && currentCell[1] == finalCell[1]) {
				endProgram();
			}
			// Print
			int distance = wavefrontGridManager.getGridPos(currentCell[0], currentCell[1]);
			int[] nextTile;
			int[] currentTile = currentCell;
			
			while(distance > 0){
				nextTile = wavefrontGridManager.getNextTile(currentTile);

				currentTile = new int[2];
				currentTile[0] = nextTile[0];
				currentTile[1] = nextTile[1];

				distance--;
			}
			
			return wavefrontGridManager.getNextTile(currentCell);
		}
		
		ret[0] = -1;
		ret[1] = -1;
		
		return ret;
	}
	
	// Manually set the estimated location
	public static void setLocation(int x, int y, float h) {
		estimatedLocation[0] = x;
		estimatedLocation[1] = y;
		estimatedLocation[2] = (int) h;
	}
	
	// Get the robot's current estimated location
	public static int[] getEstimatedLocation() {
		return estimatedLocation;
	}
	
	// Is there an obstacle in the next tile?
	public static boolean isObstacleInNextTile(int[] location) {
		// Check heading
		int h = location[2];
		int x = location[0];
		int y = location[1];
		
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
		
		if(gridManager.getProbability(x, y) > 0.5) 
			return true;
		else
			return false;
	}
	
	// Set final tile
	public static void setFinalTile(int x, int y) {
		finalCell[0] = x;
		finalCell[1] = y;
	}
	
	// Have we completed the occupancy grid?
	public static boolean isMapFinished() {
		for(int i = 0; i < yGridCount; i++) {
			for(int j = 0; j < xGridCount; j++) {
				if(gridManager.getGridCounterValue(j, i) == 0)
					return false;
			}
		}

		canGoToFinalCell = true;

		return true;
	}
	
	// End the program
	private static void endProgram() {
		pilotRobot.endSensors();
		
        int keyPressed = Button.getButtons();
        boolean quit = false;
        
        // Send final occupancy grid to PC
		printMessage("=== Occupancy Grid ===");
		printMessage(gridManager.printGrid());

    	// Print a message on the screen and wait for one of the button presses        
        while(quit == false){   
            switch (keyPressed) {
	            case Keys.ID_LEFT:
	                // Display the grid
	        		for(int i = 0; i < 15; i++) {
	        			System.out.println("");
	        		}
	        		
	        		gridManager.drawMap();
	        		
	        		break;
	            case Keys.ID_RIGHT:
	                // Display the probability matrix
	            	lcd.clear();
	        		System.out.print(gridManager.printGrid());
	        		break;
	        	default:
	        		quit = true;
	        		break;    			
            }
            Button.waitForAnyPress(); 
            keyPressed = Button.getButtons();
        }

		try {
			printMessage("Finished");
			server.close();
		} catch(Exception e) {
			
		}
	}
}