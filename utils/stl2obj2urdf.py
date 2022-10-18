import pymeshlab
import os
from object2urdf import ObjectUrdfBuilder
import getopt, sys
 
argumentList = sys.argv[1:]
options = "zm:"

def convert_mesh(inputfile, outputfile):
    # create a new mesh set
    ms = pymeshlab.MeshSet()
    # load a new mesh
    ms.load_new_mesh(inputfile)
    # save the current mesh with default parameters
    ms.save_current_mesh(outputfile)

def stl2urdf(path):
    dir_list = os.listdir(path)

    # convert all .dae file to .obj
    for file in dir_list:
        if '.stl' in file:
            convert_mesh(path+file, path+file.replace('.stl', '.obj'))
            
    builder = ObjectUrdfBuilder(path, urdf_prototype='../_prototype.urdf')
    builder.build_library(force_overwrite=True, decompose_concave=False, force_decompose=False, center = None)

def main():
    try:
        arguments, _ = getopt.getopt(argumentList, options)
        for currentArgument, currentValue in arguments:
            if currentArgument == "-z":
                print ("Converting stl files to urdf from zonopy ...")
                stl2urdf('../assets/zonotope/meshes_zonopy/')
            if currentArgument == "-m":
                print ("Converting stl files to urdf from matlab ...")
                stl2urdf('../assets/zonotope/meshes_matlab/')
    except getopt.error as err:
        print (str(err))

if __name__ == "__main__":
    main()