import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;


public class MapDrawer  {
	private static Brick myEV3 = BrickFinder.getDefault();
	private static GraphicsLCD lcd = myEV3.getGraphicsLCD();
	int r;
	int t;
	int cellSize = 15;
	
	public void initMap(int [][]map) {
		r = 0;
		t = 0;
		
		for (int i = 0; i < 7; i++) {
			r = 0;		
			for (int j = 0; j < 8; j++) {
				
				lcd.drawRect(j+r, i + t, cellSize, cellSize);
				
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

		for (int i = 0; i < 7; i++) {
			r = 0;		
			for (int j = 0; j < 8; j++) {
				
				if (map[i][j] > 1)
					lcd.fillRect(j+r, i + t, cellSize, cellSize);
				else
					lcd.drawRect(j+r, i + t, cellSize, cellSize);	
				
				
				r = r + cellSize;	//Set space between the rows		
			}	
			t = t + cellSize; //Set space between the columns
		}
	}
}


