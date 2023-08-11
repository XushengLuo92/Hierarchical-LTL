# import numpy as np

# # Create a sample matrix
# matrix = np.array([
#     [148, 21.350098],
# [145, 21.106741],
# [147, 20.247928],
# [145, 22.657795],
# [148, 21.278369],
# [147, 21.339671],
# [150, 21.458046],
# [146, 21.645851],
# [149, 21.615585],
# [150, 21.237824],
# [152, 22.185375],
# [150, 22.316069],
# [148, 21.132294],
# [147, 21.061217],
# [144, 21.780337],
# [145, 21.596012],
# [147, 21.218222],
# [150, 21.410828],
# [150, 20.795024],
# [146, 21.439786],

# ])

# # Calculate mean and standard deviation of each column
# column_means = np.mean(matrix, axis=0)
# column_stds = np.std(matrix, axis=0)

# print("Mean of each column:", column_means)
# print("Standard deviation of each column:", column_stds)

import json

# Path to the JSON file
file_path = 'src/lego_config.json'

# Read and load the JSON file
with open(file_path, 'r') as file:
    data = json.load(file)

# Generate a dictionary with 'id' as the key
brick_dict = {brick['id']: brick for brick in data['bricks']}

# Print the dictionary
for key, value in brick_dict.items():
    print(f"ID: {key}, Details: {value}")
