package VectorFields;

import robocode.*;
import java.lang.*;

/**
 * Robocode robot that acts as a moving goal for testing.
 */
public class VectorFieldsMovingGoal extends Robot
{
    public void run()
    {
        while(true)
        {
            ahead(580);
            turnRight(90);
        }
    }
}
