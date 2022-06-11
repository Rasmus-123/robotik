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

def move_clockwise(array, steps):
    """..."""
    return array[-steps:] + array[:-steps]

def move_counterclockwise(array, steps):
    """..."""
    return array[steps:] + array[:steps]

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

## Schritt 2 - 2 Schritte in zufällige Richtung ##

print("## Schritt 2 - 2 Schritte in zufällige Richtung ##")

probs_cw = move_clockwise(probs, 2)
probs_ccw = move_counterclockwise(probs, 2)

probs_cw = normalize(probs_cw)
probs_ccw = normalize(probs_ccw)

print_probabilities(probs_cw)
print_probabilities(probs_ccw)

## Schritt 3 - Eine Landmarke ##

print("## Schritt 3 - Eine Landmarke ##")

probs_cw = landmark(probs_cw, True)
probs_cw = normalize(probs_cw)
probs_ccw = landmark(probs_ccw, True)
probs_ccw = normalize(probs_ccw)

print_probabilities(probs_cw)
print_probabilities(probs_ccw)

## Schritt 4 - 4 Schritte in zufällige Richtung ##

print("## Schritt 4 - 4 Schritte in zufällige Richtung ##")

probs_cw_cw = move_clockwise(probs_cw, 4)
probs_ccw_cw = move_clockwise(probs_ccw, 4)
probs_cw_cw = normalize(probs_cw_cw)
probs_ccw_cw = normalize(probs_ccw_cw)

probs_cw_ccw = move_counterclockwise(probs_cw, 4)
probs_ccw_ccw = move_counterclockwise(probs_ccw, 4)
probs_cw_ccw = normalize(probs_cw_ccw)
probs_ccw_ccw = normalize(probs_ccw_ccw)

print_probabilities(probs_cw_cw)
print_probabilities(probs_cw_ccw)
print_probabilities(probs_ccw_cw)
print_probabilities(probs_ccw_ccw)

## Schritt 5 - Keine Landmarke ##

print("## Schritt 5 - Keine Landmarke ##")


probs_cw_cw = landmark(probs_cw_cw, False)
probs_cw_cw = normalize(probs_cw_cw)
probs_cw_ccw = landmark(probs_cw_ccw, False)
probs_cw_ccw = normalize(probs_cw_ccw)

probs_ccw_cw = landmark(probs_ccw_cw, False)
probs_ccw_cw = normalize(probs_ccw_cw)
probs_ccw_ccw = landmark(probs_ccw_ccw, False)
probs_ccw_ccw = normalize(probs_ccw_ccw)

print_probabilities(probs_cw_cw)
print_probabilities(probs_cw_ccw)
print_probabilities(probs_ccw_cw)
print_probabilities(probs_ccw_ccw)

## Result ##

# Mit > 0 = im Uhrzeigersinn
# Mit < 0 = gegen Uhrzeigersinn
# 4 Richtungsmöglichkeiten:
#    2 +  4 =  6
#    2 + -4 = -2
#   -2 +  4 =  2
#   -2 + -4 = -6

def results(array, first, second):
    max_prob = max(array)

    steps = first + second

    # Find all Indexes with max_prob (in case multiple fields have the same probability)
    max_indexes = [i for i in range(1, len(array) + 1) if array[i-1] == max_prob]
    start_fields = [i - steps for i in max_indexes]

    print("Der Roboter steht mit Wahrscheinlichkeit", round(max_prob, 2), "auf Feld:", max_indexes)
    print("Der Startzustand war nach", first, "und", second, "Schritten wahrscheinlich Feld:", start_fields)

results(probs_cw_cw, 2, 4)
results(probs_cw_ccw, 2, -4)
results(probs_ccw_cw, -2, 4)
results(probs_ccw_ccw, -2, -4)

print("Alle zusammen:")
probs_all = [a * b * c * d for a,b,c,d in zip(probs_cw_cw, probs_cw_ccw, probs_ccw_cw, probs_ccw_ccw)]
probs_all = normalize(probs_all)
results(probs_all, 0, 0)
