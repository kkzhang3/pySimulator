
def ojp2mat(ojpfile, matfile):
    with open(ojpfile, 'rb') as file:
        content = file.read()

    if (len(content) < 400 or (content[0] == 'M' and content[1] == 'A' and content[2] == 'T')) :
        new_content = content
    else:
        first_section = content[:200]
        second_section = content[200:400]
        
        new_content = second_section + first_section + content[400:]
    
    with open(matfile, 'wb') as file:
        file.write(new_content)

# ojp2mat('test.ojp', 'test.mat')

# from scipy.io import loadmat
# data = loadmat('./ojp2py/test.mat')
# print(data)

# annots = loadmat('./ojp2py/matlab1.mat')

# print(annots)

# print(f"{annots['STPPlotLength'] = }")

from mat4py import loadmat

filename = './ojp2py/test.mat'

# f = ptables.openFile(filename)
f = loadmat(filename)

# with h5py.File(filename, 'r') as file:
#     # Access a specific variable or dataset
#     data = file['A'][()]
#     print(data)

print(f)

