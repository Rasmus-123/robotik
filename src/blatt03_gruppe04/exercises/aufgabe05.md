Aufgabe 5
a)  Auf dem Odometrie Topic wird die berechnete Odometrie gepublished, welche die Richtung berechnete x,y-Position, die Orientierung des Roboters, die Lineargeschwindigkeit sowie Winkelgeschwindigkeit beinhaltet. Dies entspricht dabei der Prädiktion.


|           | Ort                                      | Geschwindigkeit                            | Beschleunigung          |
|-----------|------------------------------------------|--------------------------------------------|-------------------------|
| Lin. Bew. | odom.pose.position                       | odom.twist.linear                          | imu.linear_acceleration |
| Drehung   | imu.orientation && odom.pose.orientation | imu.angular_velocity && odom.twist.angular | ----------------------- |
