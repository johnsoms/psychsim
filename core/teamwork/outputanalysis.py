import sys, os


for r2 in {0.25,0.5,1.,2.,4.}:
    for r3 in {0.25,0.5,1.,2.,4.}:
        filename = str(r2)+"_"+ str(r3) + ".txt"
        path = os.getcwd() + "/rewardtrials6/"
        scorefile = open(path + filename, "r")
        line = scorefile.readline()
        total = 0
        while line:
            win = 0
            turns = int(line)
            if turns < int(sys.argv[1]):
                win = 1
            if sys.argv[2] == "t":
                total += turns
            else:
                total += win
            line = scorefile.readline()
        scorefile.close()
        print(filename)
        print("----------------------------------")
        print(total)
        print("\n\n\n")
