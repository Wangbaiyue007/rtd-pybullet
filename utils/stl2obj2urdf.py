import pymeshlab
import os
from object2urdf import ObjectUrdfBuilder

def convert_mesh(inputfile, outputfile):
    # create a new mesh set
    ms = pymeshlab.MeshSet()
    # load a new mesh
    ms.load_new_mesh(inputfile)
    # save the current mesh with default parameters
    ms.save_current_mesh(outputfile)

path = '../assets/zonotope/meshes_matlab/'
dir_list = os.listdir(path)

# convert all .dae file to .obj
for file in dir_list:
    if '.stl' in file:
        convert_mesh(path+file, path+file.replace('.stl', '.obj'))
        
builder = ObjectUrdfBuilder(path, urdf_prototype='../_prototype.urdf')
builder.build_library(force_overwrite=True, decompose_concave=False, force_decompose=False, center = None)
