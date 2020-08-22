So…
since the position out of the sensor fusion algorithm will likely be a PoseWithCovarianceStamped, the position will be transmitted as usual 3D position (x,y,z) however the rotation will be transmitted as a quaternion. 
I think it is more complicated to gain the z-axis rotation out of the quaternion than simply sending the euler values, as seen from ground station perspective.
But in practice it is only two calculations that really have to be done using trigonometric functions to gain the theta value.
As this can be easily achieved, I would rally for doing it that way, so that future generations of mars rover engineers have access to all the data that can be used to determine the rover position in space, speaking of 3D model and stuff.
So a conversion has to be added before displaying the value on the 2D map on the ground station.
Covariance gets neglected at first. I personally see no real use for this because it is really more a debugging value to determine how exact our position is. Of course we could draw cirlces around our position, but for the upcoming event this would be more trouble than worth imo.


About the ethernet_interface:

2 functions:
receiving sensory data from the bosch sensor that is passed on by the microcontroller managing it.
And passing down the pose estimation for the rover from the jetson board to ground station

Receiving ethernet happens in secondary thread. (Marcel Burda‘s udp socket implementation)

