# Aufgabe 1

## A)

### Initialisierung

Initialisiere alle Aufenthaltswahrscheinlichkeiten für die einzelnen Felder mit einer gleichverteilten Wahrscheinlichkeit von 1/16:  
```[0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625]^T```

![](_resources/figure%20-%200.png "Initialwahrscheinlichkeiten")

### Schritt 1 - Eine Landmarke ###

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.112, 0.04, 0.04, 0.112, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04]^T```

![](_resources/figure%20-%201.png "Landmarke 1")

### Schritt 2 - 2 Schritte im Uhrzeigersinn ###

Der Roboter bewegt sich zwei Zellen im Uhrzeigersinn, was bedeutet, dass die Wahrscheinlichkeiten in dem Vektor um je zwei Stellen weiter verschoben werden. Dies geschieht dabei dann 'im Kreis':  
```[0.04, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04]^T```

![](_resources/figure%20-%202.png "2 Schritte")

### Schritt 3 - Eine Landmarke ###

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.0752, 0.0269, 0.0752, 0.0752, 0.0269, 0.2105, 0.0269, 0.0752, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269]^T```

![](_resources/figure%20-%203.png "Landmarke 2")

### Schritt 4 - 4 Schritte im Uhrzeigersinn ###

Der Roboter bewegt sich vier Zellen im Uhrzeigersinn, was bedeutet, dass die Wahrscheinlichkeiten in dem Vektor um je vier Stellen weiter verschoben werden. Dies geschieht dabei dann 'im Kreis':  
```[0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0752, 0.0269, 0.2105, 0.0269, 0.0752, 0.0752, 0.0269, 0.0752, 0.0269]^T```

![](_resources/figure%20-%204.png "4 Schritte")

### Schritt 5 - Keine Landmarke ###

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.3 und jeden Eintrag auf welchem keine Landmarke steht mit 0.75. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.0349, 0.0312, 0.0873, 0.0125, 0.0873, 0.0125, 0.0873, 0.0873, 0.0125, 0.2444, 0.0312, 0.0873, 0.0349, 0.0312, 0.0873, 0.0312]^T```  

![](_resources/figure%20-%205.png "Keine Landmarke - Finales Ergebnis")

Der Roboter steht mit Wahrscheinlichkeit 0.2444 auf Feld: [10]  
Da er 6 Schritte im Uhrzeigersinn gefahren ist, ist das wahrscheinlichste Startfeld: [4]



## B)

### Initialisierung

Initialisiere alle Aufenthaltswahrscheinlichkeiten für die einzelnen Felder mit einer gleichverteilten Wahrscheinlichkeit von 1/16:  
```[0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625]^T```

![](_resources/figure%20-%200.png "Initialwahrscheinlichkeiten")

### Schritt 1 - Eine Landmarke ###

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.112, 0.04, 0.04, 0.112, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04]^T```


![](_resources/Figure-A1b-0.png "Landmarke 1")

### Schritt 2 - 2 Schritte in zufällige Richtung ###

Um zu modellieren, dass der Roboter in beide Richtungen gefahren sein könnte, werden zuerst die Wahrscheinlichkeiten aus dem Ausgangsverktor um zwei Stellen nach rechts verschoben. Ebenfalls werden die Wahrscheinlichkeiten aus dem Ausgangsvektor um zwei Stellen nach links verschoben. Dadurch erhält man folgende Vektoren:  
Clockwise: ```[0.04, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04]^T```  
Counterclockwise: ```[0.04, 0.112, 0.04, 0.112, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04]^T```

Nun werden die beiden Vektoren addiert und danach normalisiert:  
```[0.04, 0.076, 0.076, 0.076, 0.04, 0.076, 0.076, 0.076, 0.04, 0.04, 0.112, 0.04, 0.04, 0.04, 0.112, 0.04]^T```

![](_resources/Figure-A1b-1.png "2 Schritte")

## Schritt 3 - Eine Landmarke ##

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.0752, 0.051, 0.051, 0.1429, 0.0269, 0.1429, 0.051, 0.051, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269]^T```
![](_resources/Figure-A1b-2.png "Landmarke 2")


### Schritt 4 - 4 Schritte in zufällige Richtung ###

Um zu modellieren, dass der Roboter in beide Richtungen gefahren sein könnte, werden zuerst die Wahrscheinlichkeiten aus dem Ausgangsverktor um vier Stellen nach rechts verschoben. Ebenfalls werden die Wahrscheinlichkeiten aus dem Ausgangsvektor um vier Stellen nach links verschoben. Dadurch erhält man folgende Vektoren:  
Clockwise: ```[0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.051, 0.051, 0.1429, 0.0269, 0.1429, 0.051, 0.051, 0.0752, 0.0269, 0.0752, 0.0269]^T```  
Counterclockwise: ```[0.0269, 0.1429, 0.051, 0.051, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.0269, 0.0752, 0.051, 0.051, 0.1429]^T```  

Nun werden die beiden Vektoren addiert und danach normalisiert:  
```[0.051, 0.0849, 0.0631, 0.0389, 0.0752, 0.0389, 0.0631, 0.0849, 0.051, 0.0849, 0.0631, 0.0389, 0.0752, 0.0389, 0.0631, 0.0849]^T```  
![](_resources/Figure-A1b-3.png "4 Schritte")

### Schritt 5 - Keine Landmarke ###

![](_resources/Figure-A1b-4.png "Keine Landmarke - Finales Ergebnis")

Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.3 und jeden Eintrag auf welchem keine Landmarke steht mit 0.75. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:  
```[0.0241, 0.1002, 0.0745, 0.0184, 0.0888, 0.0184, 0.0745, 0.1002, 0.0241, 0.1002, 0.0745, 0.046, 0.0355, 0.046, 0.0745, 0.1002]^T```  
Der Roboter steht mit Wahrscheinlichkeit 0.1002 auf Feld: [2, 8, 10, 16]
