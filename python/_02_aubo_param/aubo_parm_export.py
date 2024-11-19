from mr_urdf_loader import loadURDF
import numpy as np
urdf_name = "/home/lza/code/04_uncalibrate_robot/aubo_description/urdf/aubo_i16.urdf"
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"] # 6,6
Mlist  = MR["Mlist"].reshape(-1) # 7,4,4
Glist  = MR["Glist"].reshape(-1) # 6,6,6
Blist  = MR["Blist"]# 6.6
np.savetxt("record/M.txt",M)
np.savetxt("record/Slist.txt",Slist)
np.savetxt("record/Mlist.txt",Mlist)
np.savetxt("record/Glist.txt",Glist)
np.savetxt("record/Blist.txt",Blist)