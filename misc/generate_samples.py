import sys
import numpy as np

f = open("samples.txt", "w")

num = int(sys.argv[1])

for i in range(num):
    f.write(str(i).zfill(6) + "\n")

f.close()
