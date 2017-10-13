f1 = open("cocktails.txt","r")
f2 = open("../config/cocktails.yaml","w")

f2.write("# Cocktails\n")
f2.write("cocktails:\n")

for line in f1:
    if (line[1] == " "):
	line = line.lstrip(" ")
	line = line.rstrip("\n")
	split = line.split(" ")
        f2.write("        - ingr:\n")
        f2.write("            type: '" + " ".join(split[2:]) + "'\n")
        f2.write("            amount: " + split[0] + "\n")
    else:
	f2.write("- cocktail:\n")
	f2.write("    name: '" + line.rstrip("\n") + "'\n")
	f2.write("    ingredients:\n")

f1.close()
f2.close()

