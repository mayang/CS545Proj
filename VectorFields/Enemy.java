package VectorFields;

import robocode.*;

public class Enemy
{
    public double bearing;
    public double x;
    public double y;

    public Enemy(double setX, double setY, double setBearing)
    {
        x = setX;
        y = setY;
        bearing = setBearing;
    }
}
