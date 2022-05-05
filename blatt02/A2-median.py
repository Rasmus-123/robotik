import statistics

numbers = [4, 4, 3, 2, 2, 9, 10, 8, 5, 8, 9, 42, 2, 2, 5, 7, 40, 6, 3, 3]

window = 5

medians = []

for i in range(0, len(numbers) - window +1):
    arr = numbers[i:i+window]
    print("Median", end='')
    print(arr, end='')
    print(" = ", end='')
    median = statistics.median(arr)
    print(median)
    medians.append(median)
    
print()
print(medians)
    