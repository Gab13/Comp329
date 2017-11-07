/*
 *  COMP329 Assignment 1
 *  Occupancy grid counter
 *  Keeps track of the number of times a grid tile has been updated
 */

public class OccupancyGridCounter {
	private int[][] grid;
	
	// Constructor
	public OccupancyGridCounter(int xGridCount, int yGridCount) {
		// Initialise grid
		grid = new int[xGridCount][yGridCount];
	}
	
	// Increments counter for a specified grid tile
	public void updateGridValue(int x, int y) {
		grid[x][y]++;
	}
	
	// Returns counter value for a specified grid tile
	public int getGridValue(int x, int y) {
		return grid[x][y];
	}
}