# Obstacle-avoiding Rectilinear Steiner Tree Construction
    Liang Li and Evangeline F. Y. Young
    Department of Computer Science and Engineering
    The Chinese University of Hong Kong
    
    
### PROBLEM DEFINITION
    障碍在二维平面上，不重叠。路径可以沿着障碍边界走。
### OUR APPROACH
    Hanan网格：所有N个点形成的N*N的网格
    在Hanan网格上从第一个点开始，迷宫搜索寻找其他的点。找到一个点后，再Backtrack寻找其他的到达这个点的最短路径，保留所有的最短路径。
    最后将路径上的交点（包括steiner点和pin），以及点之间的距离信息构成一张图，构建steniner树
    
