
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class PinpointTest {

    @Test
    public void testHeadingError() {
        // Robot Position (Meters)
        double robotX = 0;
        double robotY = 0;

        // Robot Heading (Degrees)
        double robotHeading = 0;

        // Goal Position (Meters)
        double goalX = -1.482;
        double goalY = 1.413;

        double targetAngle = Math.toDegrees(Math.atan2((goalY - robotY), (goalX - robotX)));

        double error = targetAngle - robotHeading;

        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }

        System.out.println("Robot at: (" + robotX + ", " + robotY + ")");
        System.out.println("Goal at: (" + goalX + ", " + goalY + ")");
        System.out.println("Robot heading: " + robotHeading);
        System.out.println("Calculated Error: " + error);

        assertEquals(136.4, error,0.5);
    }
}
