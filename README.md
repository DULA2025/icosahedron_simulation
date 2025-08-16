# 3D Icosahedron Physics Simulation

![Simulation GIF](icosahedron_simulation.gif)

This project is a high-performance 3D physics simulation built in Python. It features a sphere bouncing realistically inside a rotating icosahedron cage, subject to physical forces like gravity, friction, and restitution.

The entire animation is pre-calculated at high speed using a Numba-accelerated physics engine and then rendered to a high-quality, shareable animated GIF using the PyVista 3D visualization library.

---

### Features

* **Accurate Physics Engine:** Simulates realistic motion with gravity, friction, and elastic collisions.
* **Quaternion-Based Rotation:** The icosahedron's rotation is handled by `scipy`'s robust `Rotation` class for smooth, gimbal-lock-free animation.
* **High-Performance:** The core physics loop is Just-In-Time (JIT) compiled to native machine code using `numba`, allowing for thousands of complex physics frames to be calculated in seconds.
* **Offline Rendering:** The simulation is decoupled from the rendering, ensuring a perfectly smooth, high-framerate animated GIF is produced as the final output.
* **Configurable:** All key simulation parameters (duration, FPS, gravity, bounce energy, initial velocities, etc.) are located in a single `Config` class for easy tweaking and experimentation.

### Technology Stack

* **Python 3**
* [**PyVista**](https://docs.pyvista.org/): For 3D mesh creation and final rendering.
* [**NumPy**](https://numpy.org/): For all numerical and vector operations.
* [**SciPy**](https://scipy.org/): For robust, quaternion-based 3D rotation calculations.
* [**Numba**](https://numba.pydata.org/): For JIT compilation and high-performance acceleration of the physics engine.
* [**pyvistaqt**](https://github.com/pyvista/pyvistaqt): As a required backend for PyVista rendering.

---

### Setup & Usage

**1. Prerequisites**
* Python 3.8 or newer.

**2. Installation**
Open your terminal or command prompt and install the required libraries using pip:
```bash
pip install numpy pyvista scipy numba pyvistaqt
```

**3. Running the Simulation**
Save the script as `simulation.py` and run it from your terminal:
```bash
python simulation.py
```

**4. Output**
The script will first print its progress as it calculates all the physics frames. This should be very fast.
```
--- Final Icosahedron Physics Simulation & Renderer ---
Generated a perfect Icosahedron with 20 faces.
Simulating 30.0 seconds of physics at 240 FPS...
This will generate a total of 7200 frames. Please be patient...
Simulation complete in 4.51 seconds.
```
After the simulation is complete, it will render the animation.
```
Rendering animation to icosahedron_simulation_long_fast.gif...
--- Rendering complete! ---
Please open the file 'icosahedron_simulation_long_fast.gif' to view the final animation.
```
The final animated GIF will be saved in the same directory as the script.

### Customization
To change the simulation, simply modify the values in the `Config` class at the top of the script. You can easily change the duration, speed, gravity, bounce physics, and initial conditions of the sphere.
