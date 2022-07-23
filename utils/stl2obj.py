import pymeshlab
import os

def convert_mesh(inputfile, outputfile):
    # create a new mesh set
    ms = pymeshlab.MeshSet()
    # load a new mesh
    ms.load_new_mesh(inputfile)
    # save the current mesh with default parameters
    ms.save_current_mesh(outputfile)

path = '../zonotope/meshes/'
dir_list = os.listdir(path)

# convert all .dae file to .obj
for file in dir_list:
    if '.stl' in file:
        convert_mesh(path+file, path+file.replace('.stl', '.obj'))