package VectorFields;
import robocode.*;
//import java.awt.Color;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * Goal - Static Goal basicaly the same as sitting duck...minus I guess the persistance. Just a dummy holder for the goal.
 * so it can distinguish goals and obss
 */
public class Goal extends Robot
{
	public void run() {
		while(true) {
			turnGunLeft(360);
			// ahead(580);
            // turnRight(90);
			}
	}
}
