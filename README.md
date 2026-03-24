# Info
Inits vl53l5cx and configures to have the following settings:
- Resolution 4x4
- Ranging period 10 Hz

# Notes
You may need to increase the main stack size if there is a stack overflow.

Run `idf.py menuconfig`. Go to Component Config -> ESP System settings and increase the Main task stack size to at least `7168`.