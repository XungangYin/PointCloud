#--coding: utf-8--

import sys
from sys import argv
script, pn_file=argv
LIB_PATH="/usr/local/lib"
sys.path.append(LIB_PATH)
import FreeCADGui
import FreeCAD
import Part
from FreeCAD import Base

def points_trans(pn_file):
    pn=open(pn_file,'r')
    normals = []
    
    for line in pn:
        lp=line.split()
        p_and_n=[(float(lp[0]),float(lp[1]),float(lp[2])),(float(lp[3]),float(lp[4]),float(lp[5]))] #看清！这是两个三维坐标组成的列表

        n=Part.makeLine(p_and_n[0], p_and_n[1])
        normals.append(n)
    tu=Part.Compound(normals)

    FreeCADGui.showMainWindow()
    d=FreeCAD.newDocument("drawnormals")
    Part.show(tu) #只有一个图形对象
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewAxometric()   



    
def main():
    points_trans(pn_file)

main()
