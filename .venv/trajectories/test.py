import json
import math
import numpy as np



with open('linear_spline_example.json') as json_data:
    data_dict = json.load(json_data)

print("taille data_dict = ",len(data_dict))
print(data_dict["x"])

for i in range(len(data_dict["x"])):
    print("x[",i,"] = ", data_dict["x"][i][1])
