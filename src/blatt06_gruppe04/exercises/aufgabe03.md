# Aufgabe 3

## a
![picture 1](_resources/83569ab041e994bc33f27f6a9207e7d7ff8d32447bdc66596d8a7d7d08ed23e7.png)  

## b

![picture 2](_resources/128f32ef3ba2b9c1290dc62b22ca349cd05232e8c439c885c427511ef9dcf9ec.png)  
![picture 3](_resources/ce1bc16e1ed32aee9ea3dd1fada4c23fbd132101cbf334bd06750885e2ba2d66.png)  

# c
![picture 4](_resources/ef0946dd92c0592d2743c92c655c04eda1e9005f820065b8a2edc85fe7bcf95d.png)  

Die kürzesten Pfade sind alle ähnlich gut befahrbar. Bei manchen Pfaden geht der Roboter häufiger ganz knapp an einer Ecke dran vorbei, wobei man davon ausgehen könnte, dass das vielleicht nicht erwünscht ist
Grundsätzlich macht das bei den kürzesten Pfaden aber kaum einen Unterschied.  
Bei den etwas längeren Pfaden hingegen gibt es einige (z. B. außen rechts herum), bei dem der Roboter deutlich weniger nah an den Hindernissen vorbei fahren müsste, was man als befahrbarer interpretieren könnte.

# d
Der Algorithmus geht normalerweise so vor, dass er auf einen aktuellen Feld steht und sich dann das Feld in seiner Nachbarschaft auswählt, welches den geringsten Wert hat. Ist diese nun (von z. B. einem Menschen) belegt, so kann dieser nicht auf dieses Feld fahren und somit dieses auch nicht auswählen.
Stattdessen könnte dieser dann das Feld mit dem zweiten geringsten Wert auswählen und damit seine Pfadplanung fortsetzen.
Führt das dazu, dass der Roboter sich irgendwann auf einem Feld befindet, auf welchen er sich nicht weiter bewegen kann, da um das aktuelle Feld herum nicht Felder mit höheren Wert als das aktuelle, nicht befahrbare Felder oder schon befahrende Felder vorhanden sind, so muss dieser eine Art Backtracking durchführen und von einer anderen Position einen anderen Weg auswählen.