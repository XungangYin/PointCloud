from sys import argv
script, vtx_file = argv
vtx = open( vtx_file , "r" )
out = open("points.asc", "w")
i=0
for line in vtx:
    oblist = line.split()
    if oblist[0] != "#":
        out.write(line)
        i = i + 1
print i


        
        
