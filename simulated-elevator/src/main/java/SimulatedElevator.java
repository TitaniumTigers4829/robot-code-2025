public class SimulatedElevator {
    private int currentFloor;
    private int totalFloors;
    private boolean[] requests;

    public SimulatedElevator(int totalFloors) {
        this.totalFloors = totalFloors;
        this.currentFloor = 0; // Start at ground floor
        this.requests = new boolean[totalFloors];
    }

    public void requestFloor(int floor) {
        if (floor >= 0 && floor < totalFloors) {
            requests[floor] = true;
        }
    }

    public void move() {
        for (int i = 0; i < totalFloors; i++) {
            if (requests[i]) {
                while (currentFloor < i) {
                    currentFloor++;
                    System.out.println("Moving up to floor " + currentFloor);
                }
                while (currentFloor > i) {
                    currentFloor--;
                    System.out.println("Moving down to floor " + currentFloor);
                }
                System.out.println("Arrived at floor " + currentFloor);
                requests[i] = false; // Reset request after serving
            }
        }
    }

    public int getCurrentFloor() {
        return currentFloor;
    }

    public static void main(String[] args) {
        SimulatedElevator elevator = new SimulatedElevator(5);
        elevator.requestFloor(2);
        elevator.move();
        elevator.requestFloor(4);
        elevator.move();
    }
}