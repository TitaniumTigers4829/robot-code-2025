import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class SimulatedElevatorTest {
    private SimulatedElevator elevator;

    @BeforeEach
    public void setUp() {
        elevator = new SimulatedElevator();
    }

    @Test
    public void testInitialFloor() {
        assertEquals(0, elevator.getCurrentFloor());
    }

    @Test
    public void testMoveToFloor() {
        elevator.moveToFloor(3);
        assertEquals(3, elevator.getCurrentFloor());
    }

    @Test
    public void testRequestHandling() {
        elevator.requestFloor(2);
        elevator.update();
        assertEquals(2, elevator.getCurrentFloor());
    }

    @Test
    public void testEdgeCase() {
        elevator.moveToFloor(-1);
        assertEquals(0, elevator.getCurrentFloor());
    }

    @Test
    public void testMultipleRequests() {
        elevator.requestFloor(1);
        elevator.requestFloor(3);
        elevator.update();
        assertEquals(3, elevator.getCurrentFloor());
    }
}