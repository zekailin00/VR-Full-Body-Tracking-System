# Full Body Tracking

The goal of this project is to do full body tracking that when IMU sensors are placed on the body parts of a human being, tracking data can be streamed to a computer and extracted to visualize the movement of the body parts.

## Architecture 

Three subsystems
1. The sensor network that records and posts data to a server in real time.
2. The server that computes the pose data from the received IMU data.
3. The Unity program that visualizes the pose data. 

See ```ArchitectureDiagram.drawio``` for more detail.


## Trackers

* Power and I2C between ESP8266 and the IMU sensor.
* A case for holding the PCB and allowing us to wrap the trackers around the body parts more easily. There are 

See `hardware` and `software\firmware` for more detail.

## Algorithm

* Input from IMU sensors and VR headset sensors.
* Calculate local orientation data for each human body part.
    * Inclination measurement from accelerometer to calculate roll, yaw. 
    * Upper and lower parts of four limbs, head, and body are tracked.
* Output to Unity to be visualized.

See `software\server\algorithm` for more detail.

## Server
* Facilitate data transmission
    * Flask Server with HTTP protocol
    * Receive IMU data 
    * Send pose data
* Run the algorithm 
    * Concurrent with HTTP service
    * Producer and Consumer model
    * Store data in buffers
* Two modes of operation
    * Pose Estimation and Calibration
    * Change mode by HTTP request

See `software\server\app` for more detail.

## Unity Visualization
* The character is controlled by a script in the scene that requests the pose data from the server and assigns the values back to the joints of the character. 
* The mirror is placed there so the person wearing the VR headset can still see himself in the virtual environment.
* A selected portion of the character’s joints are computed by the pose estimation algorithm from the tracking data.

See `software/unity` for more detail.