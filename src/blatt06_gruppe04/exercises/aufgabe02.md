# Aufgabe 2

## a
Die Roboter fahren in unserem Fall immer rechts-herum

#### Bug 1
![picture 2](_resources/94c155c4ecca001d15b00535b6d7732930fbdc1af391cd7254b8fa42f9497009.png)  

#### Bug 3
![picture 3](_resources/90e97c07cf3200e9c9869c7ef90d7734bf35d037e8e61ef3ffa85ed7550174ec.png)  


## b
![picture 1](_resources/0b890eb18b2254587947a9080c84a7c9e071ae5ecb2e5cd4e3a40cacfe46bd61.png)  

## c
Für die Algorithmen muss bestimmt werden wie weit der Roboter von dem Ziel entfernt ist. Dafür muss die Position des Roboters und die Position des Ziels relativ zueinander bekannt sein. Dafür ist nicht zwangsläufig eine Karte notwendig, allerdings muss auch der Pfad den der Roboter bereits zurück gelegt hat irgendwie eingeordnet werden, wofür eine Karte sinnvoll wäre, welche aber auch relativ zur Startposition sein darf.
Es muss irgendwie möglich sein die Bewegung des Roboters zu bestimmen. Dafür lässt sich z. B. die Odometry verwenden, mit welcher sich die relative Roboterbewegung bestimmen lässt. (Dies wird benötigt um den zurückgelegten Pfad einzutragen und um den aktuellen Abstand zum Ziel zu bestimmen)
Ebenfalls muss der Roboter bestimmen können, ob dieser auf ein Hindernis trifft. Dafür eignet sich z. B. ein Laserscan, allerdings ließe sich dafür z. B. auch ein Abstandsensor mit Schall oder auch eine Stereokamera verwenden.

Mithilfe dieser Dinge lässt sich dann der Abstand vom Roboter zum Ziel bestimmen, bestimmen ob der Roboter gerade auf ein Hindernis trifft und ebenso welche Strecke der Roboter bereits zurückgelegt hat. All diese Dinge werden für die Algorithmen benötigt.