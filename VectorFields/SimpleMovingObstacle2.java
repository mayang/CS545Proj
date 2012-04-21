package VectorFields;

import robocode.*;

public class SimpleMovingObstacle2 extends Robot
{
    public void run()
    {
        while (true)
        {
            ahead(100);
            turnRight(180);
        }
    }
}
