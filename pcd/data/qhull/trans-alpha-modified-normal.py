#--coding:utf-8--
import time
t = time.time()

import Part
from FreeCAD import Base

#下面两个绝对路径需要根据自己电脑上面 pm 库的路径进行调整。
vtx = open("/home/wl/Documents/Projects/PMProject/pm/tests/data/qhull/points.asc" , "r" )
out = open("/home/wl/Documents/Projects/PMProject/pm/tests/data/qhull/surface-out" , "r")
pn = open("/home/wl/Documents/Projects/PMProject/pm/tests/data/qhull/normal-out" , "r")

def drawdelaunay(vertices,faces, pn):
    d=FreeCAD.newDocument("drawdelaunay")
    points_cloud=[] 
    triang_faces=[]
    normals = []

    for vt in vertices:       
        o=Part.Vertex(vt)
        points_cloud.append(o)

    for ft in faces:
        eg1 = Part.makeLine(vertices[ft[0]],vertices[ft[2]])
        eg2 = Part.makeLine(vertices[ft[2]],vertices[ft[1]])
        eg3 = Part.makeLine(vertices[ft[1]],vertices[ft[0]])
        wire = Part.Wire([eg1, eg2, eg3])
        face = Part.Face(wire)
        triang_faces.append(face)

    for line in pn:
        lp=line.split()
        p_and_n=[(float(lp[0]),float(lp[1]),float(lp[2])),(float(lp[3]),float(lp[4]),float(lp[5]))] #看清！这是两个三维坐标组成的列表
        n=Part.makeLine(p_and_n[0], p_and_n[1])
        normals.append(n)

    tu01=Part.Compound(points_cloud)
    tu02=Part.Compound(triang_faces)
    tu03=Part.Compound(normals)
#    FreeCADGui.showMainWindow()
   
    Part.show(tu01)
    Part.show(tu02)
    Part.show(tu03)
    
    FreeCADGui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewTop()
    FreeCADGui.getDocument("drawdelaunay").getObject("Shape001").DisplayMode = "Shaded"

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

#
def input_out(out):
    faces = []
    for line in out:
        oblist = line.split()
        face = [int(x) for x in oblist]
        faces.append(face)
                                          
    return faces

vertices = input_vtx(vtx)
faces = input_out(out)
drawdelaunay(vertices, faces, pn)

