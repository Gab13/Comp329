import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;


public class MapDrawer  {
	private static Brick myEV3 = BrickFinder.getDefault();
	private static GraphicsLCD lcd = myEV3.getGraphicsLCD();
	int r;
	int t;
	int cellSize = 15;
	private OccupancyGridManager gridManager;
	
	public MapDrawer(OccupancyGridManager manager) {
		gridManager = manager;
	}
	
	public void initMap(int [][]map) {
		r = 0;
		t = 0;

		/*
		for (int i = 0; i < 7; i++) {
			r = 0;		
			for (int j = 0; j < 8; j++) {
				
				if ((i != 0) && (j != 0) && (i != 6) && (j != 7))
					lcd.drawRect(i + t, j + r, cellSize, cellSize);
				else lcd.fillRect(i + t, j + r, cellSize, cellSize);
				
				
				r = r + cellSize;	//Set space between the rows		
			}	
			t = t + cellSize; //Set space between the columns
		}
		*/
		
		for(int i = 0; i < (map[0].length + 2); i++) {
			r = 0;
			
			for(int j = 0; j < (map.length + 2); j++) {
				if(i == 0 || i == map[0].length + 1 || j == 0 || j == map.length + 1) {
					lcd.fillRect(j + r, i + t, cellSize, cellSize);
				} else {
					if(gridManager.getProbability(j - 1, i - 1) > 0.5) {
						lcd.fillRect(j + r, i + t, cellSize, cellSize);
					} else {
						lcd.drawRect(j + r, i + t, cellSize, cellSize);
					}
				}
				
				r = r + cellSize;	//Set space between the rows
			}
			t = t + cellSize; //Set space between the columns
		}
		
	}
	
	//For an individual cell
	public void updateMap(int [][]map, int i,int j) {
		r = 0;
		t = 0;
		
		if (map[i][j] != 0)
			lcd.fillRect(j+r, i + t, 20, 20);
		else
			lcd.drawRect(j+r, i + t, 20, 20);
	}	
	
	public void printMap(int [][]map) {
		r = 0;
		t = 0;

		lcd.clear();
		
		for(int i = 0; i < (map[0].length + 2); i++) {
			r = 0;
			
			for(int j = 0; j < (map.length + 2); j++) {
				if(i == 0 || i == map[0].length + 1 || j == 0 || j == map.length + 1) {
					lcd.fillRect(j + r, i + t, cellSize, cellSize);
				} else {
					if(gridManager.getProbability(j - 1, i - 1) > 0.5) {
						lcd.fillRect(j + r, i + t, cellSize, cellSize);
					} else {
						lcd.drawRect(j + r, i + t, cellSize, cellSize);
					}
				}
				
				r = r + cellSize;	//Set space between the rows
			}
			t = t + cellSize; //Set space between the columns
		}
		
		/*
		for (int i = 0; i < 7; i++) {
			r = 0;		
			for (int j = 0; j < 8; j++) {
				
				if ((i == 0) || (j == 0) || (i == 6) || (j == 7))
					lcd.fillRect(i + t, j + r, cellSize, cellSize);
				else {
					if (map[j-1][i-1] != 0)  
						lcd.fillRect(i + t, j + r, cellSize, cellSize);
					else
						lcd.drawRect(i + t, j + r, cellSize, cellSize);
				}
					
				
				
				r = r + cellSize;	//Set space between the rows
			}	
			t = t + cellSize; //Set space between the columns
		}
		*/
	}
}

