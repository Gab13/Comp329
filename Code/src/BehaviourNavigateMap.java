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
		if(!Assignment.isMapFinished() && Assignment.canGoForward()) {
			//Assignment.printMessage("Moving...");
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
		
		if (pilotRobot.getColour() == 2) {
			Assignment.printMessage("Blue detected");
			//Assignment.printMessage(Assignment.getPose());
		}
		
		int[] estimatedLocation = Assignment.getEstimatedLocation();
		int h = estimatedLocation[2]; // Heading
		int rotate;
		int distance;
		
		// Get target cell
		int[] unexplored = Assignment.getUnexploredGridCell(estimatedLocation, false);
		
		// Check estimated location != unexplored
		if(estimatedLocation[0] == unexplored[0] && estimatedLocation[1] == unexplored[1]) {
			// Equal, get a new location
			unexplored = Assignment.getUnexploredGridCell(estimatedLocation,  true);
		}
		
		// Use wavefront manager to get another tile to visit
		int[] nextCell = Assignment.getNextCell(estimatedLocation);

		// Where is the next cell in comparison with the current one?
		Assignment.printMessage("Comparing [" + estimatedLocation[0] + "," + estimatedLocation[1] + "] with [" + nextCell[0] + "," + nextCell[1] + "]");
		
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
		
		pilotRobot.getPilot().travel(distance);
		
		while(!suppressed && pilotRobot.getPilot().isMoving()) {
			Thread.yield();
		}
		
		pilotRobot.getPilot().stop();
		
		Assignment.setLocation(nextCell[0], nextCell[1]);

	}
}