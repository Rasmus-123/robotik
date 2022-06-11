# A1a


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

def print_probabilities(array):
    """Simple Output"""
    
    print([round(x, 2) for x in array])
    #for i in range(1, len(array) + 1):
    #    print(i, ": ", round(array[i-1], 2), sep='')

### Start ###

# Set Array with starting probabilities
probs = [1/16] * 16

## Schritt 1 - Eine Landmarke ##

print("## Schritt 1 - Eine Landmarke ##")

probs = landmark(probs, True)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 2 - 2 Schritte im Uhrzeigersinn ##

print("## Schritt 2 - 2 Schritte im Uhrzeigersinn ##")

probs = probs[-2:] + probs[:-2]

print_probabilities(probs)

## Schritt 3 - Eine Landmarke ##

print("## Schritt 3 - Eine Landmarke ##")

probs = landmark(probs, True)

probs = normalize(probs)

print_probabilities(probs)

## Schritt 4 - 4 Schritte im Uhrzeigersinn ##

print("## Schritt 4 - 4 Schritte im Uhrzeigersinn ##")

probs = probs[-4:] + probs[:-4]

print_probabilities(probs)

## Schritt 5 - Keine Landmarke ##

print("## Schritt 5 - Keine Landmarke ##")

probs = landmark(probs, False)

probs = normalize(probs)

print_probabilities(probs)

## Result ##

max_prob = max(probs)

# Find all Indexes with max_prob (in case multiple fields have the same probability)
max_indexes = [i for i in range(1, len(probs) + 1) if probs[i-1] == max_prob]
start_fields = [i - 6 for i in max_indexes]

print("Der Roboter steht mit Wahrscheinlichkeit", round(max_prob, 2), "auf Feld:", max_indexes)
print("Da er 6 Schritte im Uhrzeigersinn gefahren ist, ist das wahrscheinlichste Startfeld:", start_fields)