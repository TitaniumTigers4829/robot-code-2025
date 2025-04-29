# LeadAcidBatterySim

This project provides a simulation of a **Lead-Acid Battery** using a **two-RC-branch Thévenin model**. It models the internal dynamics of the battery, including its **State of Charge (SOC)**, **Open Circuit Voltage (OCV)**, and **polarization effects** over time.

### Features
- **State of Charge (SOC)**: Tracks the battery's energy level and updates it based on the current draw.
- **Open Circuit Voltage (OCV)**: Models the battery's voltage when not under load, varying with the SOC.
- **Polarization Effects**: Simulates both short-term and long-term polarization with a two-RC branch model.
- **Terminal Voltage**: Calculates the voltage available to the robot based on OCV, internal resistance, and polarization effects.
- **Real-Time Simulation**: Updates at a fixed time interval (e.g., 20 ms), integrating seamlessly with RoboRioSim for realistic voltage readings.

---

## How It Works

The `LeadAcidBatterySim` class models the battery using a simplified two-RC Thévenin model. Here's a breakdown of the key components:

### 1. **State of Charge (SOC)**:
   - **SOC** represents the remaining charge in the battery, ranging from 0% (empty) to 100% (full). The battery discharges over time as current is drawn from it, and SOC decreases accordingly.
   
### 2. **Open Circuit Voltage (OCV)**:
   - The **OCV** is the voltage that the battery would have when it is not supplying any current. This value is approximately 11.8V at 0% SOC and increases linearly to 12.7V at 100% SOC.

### 3. **RC Branches (Polarization)**:
   - The battery's internal resistance and polarization effects are modeled using two RC circuits. These circuits simulate the delay and voltage drop due to the battery's internal chemistry. One branch simulates short-term polarization, and the other simulates long-term effects.

### 4. **Terminal Voltage**:
   - The **terminal voltage** is the voltage seen by the robot. It is affected by the OCV, internal resistance, and polarization effects. This value is updated during each simulation step and is used to simulate how much power the robot receives from the battery.

---

## Setup and Usage

To integrate the `LeadAcidBatterySim` into your FRC robot simulation, follow these steps:

### 1. **Include Dependencies**
Ensure your project includes WPILib’s simulation libraries to interact with `RoboRioSim`.

### 2. **Create a LeadAcidBatterySim Object**
```java
LeadAcidBatterySim batterySim = new LeadAcidBatterySim(
    capacityAh,    // Nominal capacity in amp-hours (e.g., 7 Ah)
    r0,            // Internal resistance in ohms (Ω)
    rp1, cp1,      // First RC branch (resistance in ohms and capacitance in farads)
    rp2, cp2       // Second RC branch (resistance in ohms and capacitance in farads)
);
```

### 3. **Register SimMechanisms**
If you have mechanisms (e.g., motors) that draw current from the battery, you can register them with the battery simulator.

```java
batterySim.addMechanism(mechanism);
```

This will track the current draw from the mechanism and contribute to the overall battery load.

### 4. **Update the Battery Simulation**
In your robot’s periodic update loop (e.g., `robotPeriodic` or `teleopPeriodic`), call the `update` method of the `LeadAcidBatterySim` class to simulate battery behavior.

```java
batterySim.update(dt);  // dt is the time step in seconds (e.g., 0.02 for 20 ms)
```

### 5. **Get the Last Voltage**
To get the most recently simulated battery voltage, use the `getLastVoltage` method. This value can be used to monitor the battery voltage or to control your robot's systems based on available power.

```java
Voltage voltage = batterySim.getLastVoltage();
System.out.println("Battery Voltage: " + voltage.toString());
```

### 6. **Unregister Mechanisms**
If a mechanism is no longer drawing current, you can remove it from the simulation.

```java
batterySim.removeMechanism(mechanism);
```

---

## Example Usage

```java
public class Robot extends TimedRobot {
    private LeadAcidBatterySim batterySim;

    @Override
    public void robotInit() {
        // Initialize battery simulation with example parameters
        batterySim = new LeadAcidBatterySim(
            7.0,     // Nominal capacity (7 Ah)
            0.05,    // Internal resistance (0.05 Ω)
            0.3, 0.001,  // First RC branch (0.3 Ω, 0.001 F)
            2.0, 0.01    // Second RC branch (2.0 Ω, 0.01 F)
        );
    }

    @Override
    public void robotPeriodic() {
        // Update battery simulation with a time step of 20 ms
        batterySim.update(0.02);
        
        // Print the battery voltage to the console
        Voltage voltage = batterySim.getLastVoltage();
        System.out.println("Battery Voltage: " + voltage.toString());
    }

    // Example of adding and removing mechanisms
    public void addMotorSim(SimMechanism motor) {
        batterySim.addMechanism(motor);
    }
    
    public void removeMotorSim(SimMechanism motor) {
        batterySim.removeMechanism(motor);
    }
}
```

---

## Parameters

- **capacityAh**: The battery's nominal capacity in amp-hours (Ah). Typically, this would be based on the specific battery model you're simulating (e.g., 7 Ah).
- **r0**: The instantaneous resistance in ohms (Ω), which represents the battery’s internal resistance.
- **rp1, cp1**: Parameters for the first RC branch. `rp1` is the resistance (Ω) and `cp1` is the capacitance (F) for the short-term polarization.
- **rp2, cp2**: Parameters for the second RC branch. `rp2` is the resistance (Ω) and `cp2` is the capacitance (F) for the long-term polarization.

---

## Further Enhancements

- **Advanced Battery Models**: Implement more detailed models for different types of batteries, such as Lithium-Ion.
- **Temperature Effects**: Account for the effect of temperature on battery performance (e.g., internal resistance increases with temperature).
- **Dynamic Load Effects**: Incorporate dynamic load behavior, where the battery’s voltage might drop more rapidly under heavy loads.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

This README gives a solid overview of the simulation and how to integrate it with FRC code. You can adapt it to your specific needs or extend it with more advanced features.