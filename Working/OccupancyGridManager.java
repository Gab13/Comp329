/*
 *  COMP329 Assignment 1
 *  Occupancy grid manager
 *  Creates occupancy grid and updates grid tile values
 */
public class OccupancyGridManager {
	private int[][] grid;
	private OccupancyGridCounter counter;
	MapDrawer map = new MapDrawer(this);//G.R
	
	// Constructor
	public OccupancyGridManager(int xGridCount, int yGridCount) {
		// Initialise grid
		grid = new int[xGridCount][yGridCount];
		counter = new OccupancyGridCounter(xGridCount, yGridCount);
	}
	
	// Update a specified grid tile value
	public void updateGridValue(int x, int y, int value) {
		if(x >= 0 && grid.length > x) {
			if(y >= 0 && grid[x].length > y) {
				//for(int i = 0; i < x; i++) {
					//counter.updateGridValue(i, y);
				//}
				grid[x][y] += value;
				counter.updateGridValue(x, y);
			}
		}
	}
	
	// Get a specified grid tile value
	public int getGridValue(int x, int y) {
		if(x >= 0 && grid.length > x)
			if(y >= 0 && grid[x].length > y)
				return grid[x][y];
		
		return -1;
	}
	
	// Get a specified grid tile counter value
	public int getGridCounterValue(int x, int y) {
		if(x >= 0 && grid.length > x)
			if(y >= 0 && grid[x].length > y)
				return counter.getGridValue(x, y);
		
		return -1;
	}
	
	// Get probability of there being an obstacle in a grid tile
	public float getProbability(int x, int y) {
		if(x >= 0 && grid.length > x)
			if(y >= 0 && grid[x].length > y)
				if(grid[x][y] > 0 && counter.getGridValue(x, y) > 0)
						return grid[x][y] / counter.getGridValue(x, y);
		
		return -1;
	}
	
	public void drawMap() { //G.R
		map.initMap(grid);
	}
	public void updateMap() { //G.R
		map.printMap(grid);
	}
	
	// Print a grid
	public String printGrid() {
		String ret = "  ";
		
		int xLength = grid.length;
		int yLength = grid[0].length;
		
		ret += "Dimensions: " + xLength + "x" + yLength + "\n ";
		
		for(int k = 0; k < grid.length; k++) {
			ret += " " + k;
		}
		ret += "\n";

		for(int i = 0; i < grid[0].length; i++) {
			ret += i;

			for(int j = 0; j < grid.length; j++) {
				ret += " " + getGridValue(j, i);
			}
			ret += "\n";
		}
		
		return ret;
	}
}