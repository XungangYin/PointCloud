#--coding:utf-8--
import time
t = time.time()

import Part
from FreeCAD import Base

#下面两个绝对路径需要根据自己电脑上面 pm 库的路径进行调整。
vtx = open("/home/wl/Documents/Projects/PMProject/pm/tests/data/qhull/points.asc" , "r" )
out = open("/home/wl/Documents/Projects/PMProject/pm/tests/data/qhull/alpha-out" , "r")


def drawdelaunay(vertices,faces):
    d=FreeCAD.newDocument("drawdelaunay")
    points_cloud=[] 
    triang_faces=[]

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

    tu01=Part.Compound(points_cloud)
    tu02=Part.Compound(triang_faces)
#    FreeCADGui.showMainWindow()
   
    Part.show(tu01)
    Part.show(tu02)
    
    FreeCADGui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewTop()


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

#如果需要交换，就交换
def swap_if_need(face, face_u):
    face_fi = []
    for y in range(3):
        edge_u = face_u[0:y] + face_u[y+1:3]

        dd = [0,1,2]
        dd.pop(y)

        for z in range(3):
            edge = face[0:z] + face[z+1:3]
            if edge == edge_u:
                
                ee = [0,1,2]
                ee.pop(z)
                
                tmp0 = face[ee[0]]
                tmp1 = face[ee[1]]
                tmp2 = face[z]

                face[dd[0]] = tmp1
                face[dd[1]] = tmp0
                face[y] = tmp2
                
                return face
            edge_r = edge.reverse()
            if edge_r == edge_u:
                
                ee = [0,1,2]
                ee.pop(z)
                
                tmp0 = face[ee[0]]
                tmp1 = face[ee[1]]
                tmp2 = face[z]

                face[dd[0]] = tmp0
                face[dd[1]] = tmp1
                face[y] = tmp2
                                             
    return face
    
def input_out(out):
    alpha_face_lists = []
    faces = []
    flag = 0
    i=0
    for line in out:
        i += 1
        if (line == "+++++\n"):
            flag += 1
            if (flag % 2) == 1:
                faces = []
            else:
                if (len(faces)):
                    alpha_face_lists.append(faces)   #由于采用较大规模点云，这里可以认为 faces 中一定有元素
        else:
            oblist = line.split()
            face = [int(x) for x in oblist]
            faces.append(face)
                                          
    return alpha_face_lists

vertices = input_vtx(vtx)
alpha_face_lists = input_out(out)
for faces in alpha_face_lists:
    drawdelaunay(vertices, faces)

