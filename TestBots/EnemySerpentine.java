package TestBots;
import robocode.*;
//import java.awt.Color;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * EnemySerpentine - Test Robot to interact with the actual bots with the stuff
 */
public class EnemySerpentine extends Robot
{
	boolean turnFlag = true;
	
	/**
	 * run: EnemySerpentine's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar

		// Robot main loop
		while(true) {
			// Replace the next 4 lines with any behavior you would like
			ahead(100);
		}
	}

	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		if (turnFlag) {
			turnRight(90);
			ahead(200);
			turnRight(90);
		} else {
			turnLeft(90);
			ahead(200);
			turnLeft(90);
		}
		turnFlag = !turnFlag;
		
	}	
}
