package VectorFields;

import robocode.*;
import java.lang.*;
import java.awt.Color;

/**
 * Robocode robot that acts as a moving goal for testing
 */
public class VectorFieldsMovingGoal extends AdvancedRobot
{
    public void run()
    {
        setAllColors(Color.YELLOW);

        while(true)
        {
            ahead(580);
            execute();
            turnRight(90);
            execute();
        }
    }
}
