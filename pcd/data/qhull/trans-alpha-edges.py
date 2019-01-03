import time
t = time.time()

from sys import argv
script , vtx_file , out_file = argv
import sys
LIB_PATH="/usr/local/lib"
sys.path.append(LIB_PATH)
import FreeCADGui
import FreeCAD
import Part
from FreeCAD import Base

vtx = open( vtx_file , "r" )
out = open(out_file , "r")


def drawdelaunay(vertices,edges):
    points_cloud=[]    
    triang_edges=[]    
             
    for vt in vertices:       
        o=Part.Vertex(vt)
        points_cloud.append(o)

    for eg in edges:
        o = Part.makeLine(vertices[eg[0]],vertices[eg[1]])
        triang_edges.append(o)                                                 
           
    tu01=Part.Compound(points_cloud)
    tu02=Part.Compound(triang_edges)
    
    FreeCADGui.showMainWindow()
    d=FreeCAD.newDocument("drawdelaunay")
    Part.show(tu01)
    Part.show(tu02)
    
    
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewAxometric()

def input_vtx(vtx):
    vertices = []
    rc_times = 0;
    for line in vtx:
        rc_times += 1
        oblist = line.split()  
        if rc_times == 1:
            dim = int(oblist[0])
        elif rc_times == 2:
            num_of_p = int(oblist[0])
        elif rc_times > 2:
            vertices.append(tuple([float(i) for i in oblist]))        
    return vertices

def input_out(out):
    alpha_edge_lists = [] 
    edges = []
    flag = 0
    for line in out:        
        if (line == "+++++\n"):
            flag += 1
            if (flag % 2) == 1:
                edges = []
            else:
                if (len(edges)):
                    alpha_edge_lists.append(edges)
        else:            
            oblist = line.split()
            edge = [int(x) for x in oblist]
            edges.append(edge)
    return alpha_edge_lists

vertices = input_vtx(vtx)
alpha_edge_lists = input_out(out)
for edges in alpha_edge_lists:
    drawdelaunay(vertices,edges)
