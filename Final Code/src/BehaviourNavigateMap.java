/*
 *  COMP329 Assignment 1
 *  Navigate map behaviour
 *  Navigates the map to generate occupancy grid + visual map
 */

import lejos.robotics.subsumption.Behavior;

public class BehaviourNavigateMap implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;
	private int travelLength;
	private int travel;

	// Constructor
	public BehaviourNavigateMap(PilotRobot robot){
    	 this.pilotRobot = robot;
    }

	// Finish the action
	public void suppress(){
		suppressed = true;
	}

	// Check if the map has been completed or not
	public boolean takeControl(){
		if(!Assignment.isMapFinished() || Assignment.canGoToFinalCell) {
			//Assignment.printMessage("Moving...");
			if(Assignment.isMapFinished() && Assignment.canGoToFinalCell) {
				Assignment.printMessage("Moving to final cell...");
			}
			
			return true;
		} else {
			//Assignment.printMessage("Map is finished");
			return false;
		}
	}

	// Perform action to navigate map
	public void action() {
		// Allow this method to run
		suppressed = false;
		
		pilotRobot.getPilot().stop();
		
		Assignment.scanWithSensor();
		
		int[] estimatedLocation = Assignment.getEstimatedLocation();
		int h = estimatedLocation[2]; // Heading
		int rotate;
		int distance;
		boolean align = false;
		
		
		/*
		if(!Assignment.ObstacleInFront()) {
			pilotRobot.getPilot().forward();

			while (pilotRobot.getUltrasonicSensor() > 0.05) {		

			}

			pilotRobot.getPilot().backward();

			while (pilotRobot.getUltrasonicSensor() < 0.05) {		

			}

			pilotRobot.getPilot().stop();
			
			align = true;
		}
		*/
		
		
		if (pilotRobot.getColour() == 2) {
			Assignment.printMessage("Blue detected");
			Assignment.setFinalTile(estimatedLocation[0], estimatedLocation[1]);
			//Assignment.printMessage(Assignment.getPose());
		}
		
		// Get target cell
		int[] unexplored = Assignment.getUnexploredGridCell(estimatedLocation, false);
		
		// Check estimated location != unexplored
		if(Assignment.isObstacleInNextTile(estimatedLocation) || (estimatedLocation[0] == unexplored[0] && estimatedLocation[1] == unexplored[1])) {
			// Equal, get a new location
			unexplored = Assignment.getUnexploredGridCell(estimatedLocation,  true);

		}
		
		// Use wavefront manager to get another tile to visit
		int[] nextCell = Assignment.getNextCell(estimatedLocation);
		
		while(nextCell[0] == -1 || nextCell[1] == -1) {
			unexplored = Assignment.getUnexploredGridCell(estimatedLocation,  true);
			nextCell = Assignment.getNextCell(estimatedLocation);
		}

		// Where is the next cell in comparison with the current one?
		if(estimatedLocation[0] == nextCell[0]) {
			if(estimatedLocation[1] == (nextCell[1] + 1)) {
				// Left
				if(h >= 45 && h < 135) {
					// Facing right from original position
					rotate = 180;
					distance = Assignment.GRID_WIDTH;
					
				} else if(h >= 135 && h < 225) {
					// Facing back towards original position
					rotate = 90;
					distance = Assignment.GRID_LENGTH;
					
				} else if(h >= 225 && h < 315) {
					// Facing left
					rotate = 0;
					distance = Assignment.GRID_WIDTH;
					
				} else {
					// Facing forwards
					rotate = -90;
					distance = Assignment.GRID_LENGTH;
					
				}
				
			} else {
				// Right
				if(h >= 45 && h < 135) {
					// Facing right from original position
					rotate = 0;
					distance = Assignment.GRID_WIDTH;
					
				} else if(h >= 135 && h < 225) {
					// Facing back towards original position
					rotate = -90;
					distance = Assignment.GRID_LENGTH;
					
				} else if(h >= 225 && h < 315) {
					// Facing left
					rotate = 180;
					distance = Assignment.GRID_WIDTH;
					
				} else {
					// Facing forwards
					rotate = 90;
					distance = Assignment.GRID_LENGTH;
					
				}
				
			}
		} else {
			if(estimatedLocation[0] == (nextCell[0] - 1)) {
				// Up
				if(h >= 45 && h < 135) {
					// Facing right from original position
					rotate = -90;
					
					distance = Assignment.GRID_WIDTH;
					
				} else if(h >= 135 && h < 225) {
					// Facing back towards original position
					rotate = 180;
					distance = Assignment.GRID_LENGTH;
					
				} else if(h >= 225 && h < 315) {
					// Facing left
					rotate = 90;
					distance = Assignment.GRID_WIDTH;
					
				} else {
					// Facing forwards
					rotate = 0;
					distance = Assignment.GRID_LENGTH;
					
				}
				
			} else {
				// Down
				if(h >= 45 && h < 135) {
					// Facing right from original position
					rotate = 90;
					distance = Assignment.GRID_WIDTH;
					
				} else if(h >= 135 && h < 225) {
					// Facing back towards original position
					rotate = 0;
					distance = Assignment.GRID_LENGTH;
					
				} else if(h >= 225 && h < 315) {
					// Facing left
					rotate = -90;
					distance = Assignment.GRID_WIDTH;
					
				} else {
					// Facing forwards
					rotate = 180;
					distance = Assignment.GRID_LENGTH;
					
				}
				
			}
		}
		
		pilotRobot.getPilot().rotate(rotate);
		while(!suppressed && pilotRobot.getPilot().isMoving()) {
			Thread.yield();
		}
		
		/*
		if(align == true) {
			pilotRobot.getPilot().rotate(rotate);
			while(!suppressed && pilotRobot.getPilot().isMoving()) {
				Thread.yield();
			}
			pilotRobot.getPilot().stop();
			Assignment.Align();
			align = false;
		} else {
			pilotRobot.getPilot().rotate(rotate);
			while(!suppressed && pilotRobot.getPilot().isMoving()) {
				Thread.yield();
			}
		}
		*/
		
		// Update heading after rotation
		if(h >= 45 && h < 135) {
			// Facing right from original position
			if(rotate == 90)
				h = 180;
			else if(rotate == -90)
				h = 0;
			else if(rotate == 180)
				h = 270;
			
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			if(rotate == 90)
				h = 270;
			else if(rotate == -90)
				h = 90;
			else if(rotate == 180)
				h = 0;
			
		} else if(h >= 225 && h < 315) {
			// Facing left
			if(rotate == 90)
				h = 0;
			else if(rotate == -90)
				h = 180;
			else if(rotate == 180)
				h = 90;
			
		} else {
			// Facing forwards
			if(rotate == 90)
				h = 90;
			else if(rotate == -90)
				h = 270;
			else if(rotate == 180)
				h = 180;
			
		}
		
		int[] newEstimatedLocation = {estimatedLocation[0], estimatedLocation[1], h};
		
		if(!Assignment.isObstacleInNextTile(newEstimatedLocation)) {
			pilotRobot.getPilot().travel(distance);
			
			while(!suppressed && pilotRobot.getPilot().isMoving()) {
				Thread.yield();
			}
			
			Assignment.setLocation(nextCell[0], nextCell[1], h);
		} else {
			Assignment.setLocation(estimatedLocation[0], estimatedLocation[1], h);
		}
		
		pilotRobot.getPilot().stop();


	}
}