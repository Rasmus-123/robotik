# A1a

import matplotlib.pyplot as plt

def landmark(array, found: bool):
    """Update Probabilities if landmark has been found (or not found)"""
    LANDMARKS = [1, 4, 6, 9, 13]

    landmark_factor = 0.7
    no_landmark_factor = 0.25

    if not found:
        landmark_factor = 1 - landmark_factor
        no_landmark_factor = 1 - no_landmark_factor

    for i in LANDMARKS:
        array[i-1] *= landmark_factor

    for i in [x for x in range(1, len(array) + 1) if x not in LANDMARKS]:
        array[i-1] *= no_landmark_factor

    return array


def normalize(array):
    """Normalisieren eines Zahlenarrays"""
    eta = 1 / sum(array)
    return [i * eta for i in array]

title = 0

def print_probabilities(array):
    global title
    """Simple Output"""
    
    # plot probabilities as bar graph and save it as png
    plt.bar(range(1, 17), array)
    plt.savefig(f"figure - {title}.png")
    plt.clf()
    title += 1

    print([round(x, 4) for x in array], "^T", sep='')

### Start ###

# Set Array with starting probabilities
probs = [1/16] * 16

print("Initialisiere alle Aufenthaltswahrscheinlichkeiten für die einzelnen Felder mit einer gleichverteilten Wahrscheinlichkeit von 1/16:")
print_probabilities(probs)

## Schritt 1 - Eine Landmarke ##

print("\n## Schritt 1 - Eine Landmarke ##\n")

probs = landmark(probs, True)

probs = normalize(probs)

print("Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:")
print_probabilities(probs)

## Schritt 2 - 2 Schritte im Uhrzeigersinn ##

print("\n## Schritt 2 - 2 Schritte im Uhrzeigersinn ##\n")

probs = probs[-2:] + probs[:-2]

print("Der Roboter bewegt sich zwei Zellen im Uhrzeigersinn, was bedeutet, dass die Wahrscheinlichkeiten in dem Vektor um je zwei Stellen weiter verschoben werden. Dies geschieht dabei dann 'im Kreis':")
print_probabilities(probs)

## Schritt 3 - Eine Landmarke ##

print("\n## Schritt 3 - Eine Landmarke ##\n")

probs = landmark(probs, True)

probs = normalize(probs)

print("Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.7 und jeden Eintrag auf welchem keine Landmarke steht mit 0.25. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:")
print_probabilities(probs)

## Schritt 4 - 4 Schritte im Uhrzeigersinn ##

print("\n## Schritt 4 - 4 Schritte im Uhrzeigersinn ##\n")

probs = probs[-4:] + probs[:-4]

print("Der Roboter bewegt sich vier Zellen im Uhrzeigersinn, was bedeutet, dass die Wahrscheinlichkeiten in dem Vektor um je vier Stellen weiter verschoben werden. Dies geschieht dabei dann 'im Kreis':")
print_probabilities(probs)

## Schritt 5 - Keine Landmarke ##

print("\n## Schritt 5 - Keine Landmarke ##\n")

probs = landmark(probs, False)

probs = normalize(probs)

print("Multipliziere jeden Eintrag, auf welchem eine Landmarke steht mit 0.3 und jeden Eintrag auf welchem keine Landmarke steht mit 0.75. Danach wird der Vektor normalisiert, sodass alle Wahrscheinlichkeiten addiert 1 ergeben:")
print_probabilities(probs)

## Result ##

max_prob = max(probs)

# Find all Indexes with max_prob (in case multiple fields have the same probability)
max_indexes = [i for i in range(1, len(probs) + 1) if probs[i-1] == max_prob]
start_fields = [i - 6 for i in max_indexes]

print("Der Roboter steht mit Wahrscheinlichkeit", round(max_prob, 4), "auf Feld:", max_indexes)
print("Da er 6 Schritte im Uhrzeigersinn gefahren ist, ist das wahrscheinlichste Startfeld:", start_fields)