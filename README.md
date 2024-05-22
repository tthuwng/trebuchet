### PHYS-360 Trebuchet Simulator

To run the Trebuchet Simulator, follow these steps:

#### Prerequisites

Ensure you have Python, pip and Flask installed on your system. To install Flask

```bash
pip install flask
```

#### Usage

1. **Start the Flask Server:**

   ```bash
   python app.py
   ```

   This command starts the Flask server, and you should see output indicating that the server is running, typically on `http://localhost:5000/`.

2. **Access the Simulator:**
   Open a web browser and go to `http://localhost:5000/`. This will load the Trebuchet Simulator interface.

3. **Interact with the Simulator:**

   - Set the parameters like mass of the counterweight, mass of the projectile, lengths of the arms, and launch angle.
   - Click the **Simulate** button to start the simulation. The trajectory of the projectile will be displayed on the canvas.
   - Adjust the **Speed** slider to control the simulation speed.

4. **Modify Parameters and Re-run:**
   You can change the parameters and click **Simulate** again to see how different settings affect the projectile's trajectory.
