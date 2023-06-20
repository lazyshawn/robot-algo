import numpy as np

def read_data_from_txt(fname, **readFlag):
    '''
    @brief: 从文本文件中读取数据，失败返回空 np.empty(0)
    @param: fname - 文件路径
    '''
    try:
        points = np.loadtxt(fname, **readFlag)
    except IOError:
        print("File is not accessible: ", fname)
        points = np.empty(0)
    return points

if __name__ == "__main__":
    print("module name: matrixIO")

