# Simulated Elevator Project

This project simulates an elevator system using the WPI library. It includes a main class for the elevator simulation and unit tests to ensure functionality.

## Project Structure

```
simulated-elevator
├── src
│   ├── main
│   │   ├── java
│   │   │   └── SimulatedElevator.java
│   │   └── resources
│   └── test
│       ├── java
│       │   └── SimulatedElevatorTest.java
│       └── resources
├── lib
│   └── wpi-library.jar
├── build.gradle
└── README.md
```

## Setup Instructions

1. **Clone the repository**:
   ```
   git clone <repository-url>
   cd simulated-elevator
   ```

2. **Add the WPI library**:
   Ensure that the `wpi-library.jar` file is located in the `lib` directory.

3. **Build the project**:
   Use Gradle to build the project:
   ```
   ./gradlew build
   ```

4. **Run the simulation**:
   Execute the main class to start the elevator simulation:
   ```
   ./gradlew run
   ```

## Usage

- The elevator can be controlled through method calls to simulate requests and movements.
- Unit tests are provided to validate the functionality of the elevator system.

## Examples

Refer to the `SimulatedElevator.java` file for examples of how to initialize and use the elevator simulation.