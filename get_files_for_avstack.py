from os import listdir
from os.path import isfile, join
mypath = "./build/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

ofiles = set()
sufiles = set()
for file in onlyfiles:
    if file.endswith(".su"):
        sufiles.add(file[:-3])
    elif file.endswith(".o"):
        ofiles.add(file[:-2])

n = 0
for file in ofiles:
    if file in sufiles:
        n += 1
        print(mypath + file+ ".o", end=" ")

print()
