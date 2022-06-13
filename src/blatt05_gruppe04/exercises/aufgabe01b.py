import matplotlib.pyplot as plt

# A1b

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

def move_uncertain(array, steps):
    """
    Idee:
      - 50/50 Chance auf beide Richtungen
      - Also probs1 = imUhrzeigersinn(probs)
      - Also probs2 = gegenUhrzeigersinn(probs)
      - Also probs = probs1 * probs2
    """
    clockwise = array[-steps:] + array[:-steps]
    counterclockwise = array[steps:] + array[:steps]

    print("Clockwise")
    print([round(x, 4) for x in clockwise], "^T", sep='')
    print("Counterclockwise")
    print([round(x, 4) for x in counterclockwise], "^T", sep='')
    
    return [a + b for a, b in zip(clockwise, counterclockwise)]


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
    plt.savefig(f"Figure-A1b-{title}.png")
    plt.clf()
    title += 1

    print([round(x, 4) for x in array], "^T", sep='')

### Start ###

# Set Array with starting probabilities
probs = [1/16] * 16

## Schritt 1 - Eine Landmarke ##

print("## Schritt 1 - Eine Landmarke ##\n")

probs = landmark(probs, True)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 2 - 2 Schritte in zufällige Richtung ##

print("## Schritt 2 - 2 Schritte in zufällige Richtung ##\n")

probs = move_uncertain(probs, 2)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 3 - Eine Landmarke ##

print("## Schritt 3 - Eine Landmarke ##\n")

probs = landmark(probs, True)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 4 - 4 Schritte in zufällige Richtung ##

print("## Schritt 4 - 4 Schritte in zufällige Richtung ##\n")

probs = move_uncertain(probs, 4)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 5 - Keine Landmarke ##

print("## Schritt 5 - Keine Landmarke ##\n")

probs = landmark(probs, False)

probs = normalize(probs)

print_probabilities(probs)

## Result ##

# Mit > 0 = im Uhrzeigersinn
# Mit < 0 = gegen Uhrzeigersinn
# 4 Richtungsmöglichkeiten:
#    2 +  4 =  6
#    2 + -4 = -2
#   -2 +  4 =  2
#   -2 + -4 = -6

max_prob = max(probs)

# Find all Indexes with max_prob (in case multiple fields have the same probability)
max_indexes = [i for i in range(1, len(probs) + 1) if probs[i-1] == max_prob]
start_fields = [i - 6 for i in max_indexes]

print("Der Roboter steht mit Wahrscheinlichkeit", round(max_prob, 2), "auf Feld:", max_indexes)