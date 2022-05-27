# Aufgabe 5
a)  Da der Roboter sich nur in der xy-Ebene bewegt und sich auch nur "auf" dieser drehen kann, ist nur die Position über die x- und y-Koordinaten bestimmt und die Orientierung des Roboters nur über die z-Achse. Daher könnte könnte ein Zustand so aussehen, dass der die x- und y-Koordinaten, sowieso die z-Achsen-Orientierung beinhaltet.
Auf dem Odometrie Topic wird die berechnete Odometrie gepublished, welche die berechnete x,y(,z)-Position (pose.pose.position), die Orientierung (pose.pose.orientation) des Roboters, die Lineargeschwindigkeit (twist.twist.linear), sowie Winkelgeschwindigkeit (twist.twist.angular) beinhaltet.
Auf dem IMU Topic wird die Orientierung (orientation), die Winkelgeschwindigkeit (angular_velocity), sowie die Lineargeschwindigkeit (linear_acceleration) gepublished.

Ebenfalls enthalten die Nachrichten noch Kovarianzen für die gepublishten Werte

|           | Ort                                           | Geschwindigkeit                                  | Beschleunigung          |
|-----------|-----------------------------------------------|--------------------------------------------------|-------------------------|
| Lin. Bew. | odom.pose.pose.position                       | odom.twist.twist.linear                          | imu.linear_acceleration |
| Drehung   | imu.orientation && odom.pose.pose.orientation | imu.angular_velocity && odom.twist.twist.angular | ----------------------- |
