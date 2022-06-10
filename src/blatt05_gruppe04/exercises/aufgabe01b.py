import matplotlib.pyplot as plt

# TODO: results don't make sense

# normalize probabilities
def normalize(x):
    normalization_factor = 1 / sum(x)
    for i in range(len(x)):
        x[i] *= normalization_factor
    return x

def main():
    probabilities = []

    for i in range(0,16):
        probabilities.append(1/16)

    # robot detects first landmark
    for i in range(0,16):
        probabilities[i] = 0.25

    probabilities[1-1] = 0.7
    probabilities[4-1] = 0.7
    probabilities[6-1] = 0.7
    probabilities[9-1] = 0.7
    probabilities[13-1] = 0.7

    probabilities = normalize(probabilities)

    print("Probabilities at the beginning:")
    print(probabilities)

    # open windows with simple graph visualizing the probabilities as bar chart and the title "Probabilities at the beginning"
    plt.bar(range(1,17), probabilities)
    plt.title("Probabilities at the beginning")
    plt.show()

    # robot moves two cells in unknown direction
    probabilities = probabilities[-2:] + probabilities[:-2]
    probabilities_tmp = probabilities[2:] + probabilities[:2]    
    probabilities = [probabilities[i] + probabilities_tmp[i] for i in range(len(probabilities))]

    probabilities = normalize(probabilities)


    print("Probabilities after moving two cells in unknown direction:")
    print(probabilities)
    
    
    # open windows with simple graph visualizing the probabilities as bar chart and the title "Probabilities after moving two cells in unknown direction"
    plt.bar(range(1,17), probabilities)
    plt.title("Probabilities after moving two cells in unknown direction")
    plt.show()

    # robot detects second landmark
    for i in range(0,16):
        if i == 1-1 or i == 4-1 or i == 6-1 or i == 9-1 or i == 13-1:
            probabilities[i] *= 0.7
        else:
            probabilities[i] *= 0.25
    
    probabilities = normalize(probabilities)

    print("Probabilities after detecting second landmark:")
    print(probabilities)

    # open windows with simple graph visualizing the probabilities as bar chart and the title "Probabilities after detecting second landmark"
    plt.bar(range(1,17), probabilities)
    plt.title("Probabilities after detecting second landmark")
    plt.show()
    

    # robot moves four cells in unknown direction
    probabilities = probabilities[-4:] + probabilities[:-4]
    probabilities_tmp = probabilities[4:] + probabilities[:4]
    probabilities = [probabilities[i] + probabilities_tmp[i] for i in range(len(probabilities))]

    print("Probabilities after moving four cells in unknown direction:")
    print(probabilities)

    # open windows with simple graph visualizing the probabilities as bar chart and the title "Probabilities after moving four cells in unknown direction"
    plt.bar(range(1,17), probabilities)
    plt.title("Probabilities after moving four cells in unknown direction")
    plt.show()
    

    # robot does not detect landmark
    for i in range(0,16):
        if i == 1-1 or i == 4-1 or i == 6-1 or i == 9-1 or i == 13-1:
            probabilities[i] *= 0.30
        else:
            probabilities[i] *= 0.75

    probabilities = normalize(probabilities)

    print("Probabilities after not detecting landmark:")
    print(probabilities)

    # open windows with simple graph visualizing the probabilities as bar chart and the title "Probabilities after not detecting landmark"
    plt.bar(range(1,17), probabilities)
    plt.title("Probabilities after not detecting landmark")
    plt.show()


    max_prob_pos = probabilities.index(max(probabilities))
    print("Most likely start position:", max_prob_pos-6+1)


if __name__ == "__main__":
    main()