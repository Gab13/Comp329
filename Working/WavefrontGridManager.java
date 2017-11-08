import java.util.ArrayList;

public class WavefrontGridManager {
	private int[][] grid;
	private int[] startPos, endPos;
	private int xGrid, yGrid;
	private ArrayList<ArrayList<int[]>> gridPositions;
	private OccupancyGridManager gridManager;

	public WavefrontGridManager(int xGridCount, int yGridCount, OccupancyGridManager manager) {
		grid = new int[xGridCount][yGridCount];
		xGrid = xGridCount;
		yGrid = yGridCount;
		gridManager = manager;
	}

	public void setPathTo(int[] start, int[] end) {
		if(start.length >= 2 && end.length >= 2) {
			if(start[0] < grid.length && start[1] < grid[0].length) {
				Assignment.printMessage("Starting wavefront for " + start[0] + "," + start[1] + " -> " + end[0] + "," + end[1]);
				
				startPos = start;
				endPos = end;

				grid = new int[xGrid][yGrid];

				int[] currentTile;
				ArrayList<int[]> currentTiles;
				boolean finished = false;
				int i = 1;

				gridPositions = new ArrayList<>();
				ArrayList<int[]> firstDistance = new ArrayList<>();
				firstDistance.add(end);
				gridPositions.add(firstDistance);
				grid[endPos[0]][endPos[1]] = -1;

				// Wavefront from end -> start
				Assignment.printMessage("Starting wavefront search...");
				while(!finished){
					currentTiles = getTilesAtDistance(i);
					gridPositions.add(currentTiles);
					
					if(currentTiles.size() == 0)
						break;

					for(int j = 0; j < currentTiles.size(); j++) {
						currentTile = currentTiles.get(j);

						grid[currentTile[0]][currentTile[1]] = i;

						if(currentTile[0] == startPos[0] && currentTile[1] == startPos[1]) {
							finished = true;
							break;
						} else {
							//Assignment.printMessage(currentTile[0] + " != " + startPos[0] + " && " + currentTile[1] + " != " + startPos[1]);
						}

					}

					i++;
				}

				Assignment.printMessage(gridManager.printGrid());
				Assignment.printMessage("New grid:\n" + printGrid());
			}
		}
	}

	private ArrayList<int[]> getTilesAtDistance(int distance){
		ArrayList<int[]> tiles = new ArrayList<>(), positionsArrayList = new ArrayList<>();

		// Get tiles at position distance + 1 in each direction
		int posX, posY;
		int pos[], positions[] = new int[2];

		ArrayList<int[]> tilesAtPosition = gridPositions.get(distance - 1);

		for(int i = 0; i < tilesAtPosition.size(); i++) {
			positions = new int[2];
			positions[0] = tilesAtPosition.get(i)[0];
			positions[1] = tilesAtPosition.get(i)[1];
			positionsArrayList.add(positions);
		}

		for(int[] posArray : positionsArrayList) {
			posX = posArray[0];
			posY = posArray[1];

			// Calculate tiles adjacent to [posX, posY]
			if(posX < grid.length && posY < grid[0].length) {
				if((posX + 1) < grid.length && gridManager.getProbability(posX + 1, posY) <= 0.5 && getGridPos(posX + 1, posY) == -1) {
					pos = new int[2];
					pos[0] = posX + 1;
					pos[1] = posY;
					tiles.add(pos);
				}

				if((posX - 1) >= 0 && gridManager.getProbability(posX - 1, posY) <= 0.5 && getGridPos(posX - 1, posY) == -1) {
					pos = new int[2];
					pos[0] = posX - 1;
					pos[1] = posY;
					tiles.add(pos);
				}

				if((posY + 1) < grid[0].length && gridManager.getProbability(posX, posY + 1) <= 0.5 && getGridPos(posX, posY + 1) == -1) {
					pos = new int[2];
					pos[0] = posX;
					pos[1] = posY + 1;
					tiles.add(pos);
				}

				if((posY - 1) >= 0 && gridManager.getProbability(posX, posY - 1) <= 0.5 && getGridPos(posX, posY - 1) == -1) {
					pos = new int[2];
					pos[0] = posX;
					pos[1] = posY - 1;
					tiles.add(pos);
				}
			}

		}
		
		//for(int[] tilesArray : tiles) {
			//Assignment.printMessage("[" + tilesArray[0] + "," + tilesArray[1] + "] in tiles");
		//}

		return tiles;
	}
	
	// Get a tile we need to travel to (ie, with current distance - 1)
	public int[] getNextTile(int[] currentTile){
		int currentDistance = getGridPos(currentTile[0], currentTile[1]);
		int[] nextTile = new int[2];
		
		if(currentDistance > 0){
			try {
				ArrayList<int[]> tilesAtNextDistance = gridPositions.get(currentDistance - 1);
				
				nextTile[0] = tilesAtNextDistance.get(0)[0];
				nextTile[1] = tilesAtNextDistance.get(0)[1];
				
				int i = 1;
				
				while(!isAdjacentTo(currentTile, nextTile)){
					//Assignment.printMessage("Tile [" + currentTile[0] + "," + currentTile[1] + "] is not adjacent to [" + nextTile[0] + "," + nextTile[1] + "]");
					nextTile[0] = tilesAtNextDistance.get(i)[0];
					nextTile[1] = tilesAtNextDistance.get(i)[1];
					i++;
				}
				
				Assignment.printMessage("Next tile found: [" + nextTile[0] + "," + nextTile[1] + "]");
				
				return nextTile;
				
			} catch(Exception e){
				// Error
			}
		} else
			Assignment.printMessage("Current distance is " + currentDistance);
		
		nextTile[0] = -1;
		nextTile[1] = -1;
		
		return nextTile;
	}

	// Get the distance value for a specified grid position
	public int getGridPos(int x, int y) {
		if(x < grid.length && y < grid[0].length) {
			if(grid[x][y] > 0)
				return grid[x][y];
			else if(grid[x][y] == -1)
				return 0;
		}

		return -1;
	}
	
	// Get the maximum distance value (ie, the start point)
	public int getMaxDistance(){
		return getGridPos(startPos[0], startPos[1]);
	}
	
	// Is one tile adjacent to the other?
	private boolean isAdjacentTo(int[] tileOne, int[] tileTwo){
		if(tileOne[0] == tileTwo[0]){
			if(tileOne[1] == (tileTwo[1] + 1) || tileOne[1] == (tileTwo[1] - 1)){
				return true;
			}
		} else if(tileOne[1] == tileTwo[1]){
			if(tileTwo[0] == (tileOne[0] + 1) || tileTwo[0] == (tileOne[0] - 1)){
				return true;
			}
		}
		
		return false;
	}
	
	// Print a grid
	public String printGrid() {
		String ret = " ";
		int cell;
		
		for(int k = 0; k < grid.length; k++) {
			ret += " " + k;
		}
		ret += "\n";

		for(int i = 0; i < grid[0].length; i++) {
			ret += i;

			for(int j = 0; j < grid.length; j++) {
				
				cell = getGridPos(j, i);
				
				if(cell >= 0)
					ret += "+" + cell;
				else
					ret += cell;
				
			}
			ret += "\n";
		}
		
		return ret;
	}

	/*
	// Print a grid
	public String printGrid() {
		String ret;
		
		ret = "Start: " + getGridPos(startPos[0], startPos[1]) + "\n";
		ret += "End: " + getGridPos(endPos[0], endPos[1]) + "\n";

		int cell;

		ret += " ";
		for(int k = 0; k < grid[0].length; k++) {
			ret += " " + k;
		}
		ret += "\n";

		for(int i = 0; i < grid.length; i++) {
			ret += "" + i;

			for(int j = 0; j < grid[0].length; j++) {
				cell = getGridPos(i, j);

				if(cell >= 0)
					ret += "+" + getGridPos(i, j);
				else
					ret += getGridPos(i, j);
			}
			ret += "\n";
		}
		
		return ret;
	}
	*/
}
