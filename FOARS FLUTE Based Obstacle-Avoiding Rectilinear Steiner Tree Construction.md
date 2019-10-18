# FOARS: FLUTE Based Obstacle-Avoiding Rectilinear Steiner Tree Construction

Gaurav Ajwani, Member, IEEE, Chris Chu, and Wai-Kei Mak, Member, IEEE

## I. Introduction

* 生成OARSMT的简单策略是调用一次FLUTE并使被障碍物阻挡的边合法化。

* 更好的策略是打破被障碍物阻挡的边，形成多个子树，递归调用FLUTE来局部优化子树，然后最后合并所有没有重叠的子树。

* 参考文献[1]，[3]，[4]和[10]均使用了escape graph。
  
  > [1] Y. Hu, Z. Feng, T. Jing, X. Hong, Y. Y. Ge, X. Hu, and G. Yan, “FORst: A 3-step heuristic for obstacle-avoiding rectilinear Steiner minimal tree construction,” in Proc. JICS, 2004, pp. 107–116.
  > [3] Y. Hu, T. Jing, X. Hong, Z. Feng, X. Hu, and G. Yan, “An-OARSMan: Obstacle-avoiding routing tree construction with good length performance,” in Proc. ASP-DAC, 2006, pp. 630–635.
  > [4] Y. Shi, P. Mesa, H. Yao, and L. He, “Circuit simulation based obstacle-aware
  > Steiner routing,” in Proc. DAC, 2006, pp. 385–388.
  >
  > [10] L. Li and E. F. Y. Young, “Obstacle-avoiding rectilinear Steiner tree construction,” in Proc. ICCAD, 2008, pp. 523–528.
  
  参考文献[9]利用了Delaunay triangulation based graph。
  
  > [9] I. H.-R. Jiang, S.-W. Lin, and Y.-T. Yu, “Unification of obstacle-avoiding rectilinear Steiner tree construction,” in Proc. SoCC, 2008, pp. 127–130.
  
  escape graph和Delaunay triangulation based graph都包含O(n^2)条边，其中n是pin和obstacle  corners的总数。
  参考文献[2]和[5] – [8]基于各种形式的obstacle-avoiding spanning graphs。
  [2]提出了一种OASG的形式，它仅包含线性数量的边，这在[5]中也被采用。
  
  > [2] Z. Shen, C. C. N. Chu, and Y.-M. Li, “Efficient rectilinear Steiner tree construction with rectilinear blockages,” in Proc. ICCD, 2005, pp. 38–44.
  >
  > [5] P.-C. Wu, J.-R. Gao, and T.-C. Wang, “A fast and stable algorithm for obstacle-avoiding rectilinear Steiner minimal tree construction,” in Proc. ASP-DAC, 2007, pp. 262–267.
  
  后来[6]建议在Shen的OASG中添加缺失的essential edges 。但是，它在最坏的情况下将边的数量增加到O(n^2)（实际中是O（n log n））。
  
  > [6] C.-W. Lin, S.-Y. Chen, C.-F. Li, Y.-W. Chang, and C.-L. Yang, “Obstacle-avoiding rectilinear Steiner tree construction based on spanning graphs,” IEEE Trans. Comput.-Aided Design Integr. Circuits Syst., vol. 27, no. 4, pp. 643–653, Apr. 2008.
  
  [7]，[8]提出了一种象限方法来生成具有线性边缘数量的OASG，但生成的OASG并不理想。
  
  > [7] J. Long, H. Zhou, and S. O. Memik, “An O(n log n) edge-based algorithm for obstacle-avoiding rectilinear Steiner tree construction,” in Proc. ISPD, 2008, pp. 126–133.
  >
  > [8] J. Long, H. Zhou, and S. O. Memik, “EBOARST: An efficient edgebased obstacle avoiding-rectilinear Steiner tree construction algorithm,” IEEE Trans. Comput.-Aided Design Integr. Circuits Syst., vol. 27, no. 12, pp. 2169–2182, Dec. 2008.
  
  本文中提出了一种八分法来生成具有更理想属性的O(n)条边的OASG。

## II. Overview of FOARS

* 算法分为5个步骤
  1. 构建OASG
  2. 生成MTST(minimum terminal spanning tree)和OPMST(obstacle penalized minimal spanning tree)
  3. 生成OAST(obstacle-aware Steiner tree)
  4. 生成OARSMT
  5. refinement

## III. OASG Generation

* 与之前Hai Zhou的方法一样，每个点在每个八分区域中只连接曼哈顿距离最近的点，同时要保证连接的边不会被障碍挡住。
* 以R5区域为例：所有点按照x+y从小到大排序，枚举每个点。对当前点v，在active集合的点中寻找在v的R5中且距离最近的没有被障碍阻挡的点u，连接uv并在active中删除v的R5中的所有点。
* 可以证明，对于active的点p,q，若`xp<xq`则`xp−yp≤xq− yq`。所以在平衡树中找到x坐标比xv小的第一个点再找到`x-y>xv-yv`的点，这两个点之间的点就是在R5区域的点。
* 阻挡曼哈顿路径的边界只需要考虑矩形障碍的底边界和左边界。
  1. 当前点v是障碍的左下角时将障碍底边、左边都加入障碍列表。
  2. 当v是左上角时由于其R5区域没有其他点，所以左边不会阻挡后续的连接，因此在障碍表中删除左边。
  3. 当v是右下角时，在区域`yu<yh, xa<xu, xu−yu+yh≤xb`中的点都会被阻挡，所以删除所有区域中active的点。然后在障碍表中删除底边。
* 期望时间复杂度O(nlogn)，最坏情况O(n^2)

## IV. OPMST Generation

* 使用[8]中的extended Dijkstra’s algorithm将每个corner vertex与距离最近的pin相连。形成多个以pin为根的森林，然后用extended Kruskal‘s  algorithm形成MST。
* 用上面的方法形成的MST存在一些问题，两pin直接连接的距离更短，但是由于要连接corner vertex而可能使路径变长。因此本文采用OPMST的方式来改进MTST。即删掉所有的corner vertex，将其边长加在两pin之间的距离上。

## V. OAST Generation

#### A. Partition

* 划分有两个标准：
  - 边被障碍完全阻挡：被阻挡的边是(u,v)，要经过的障碍的边是(a,b)。则将树T分成T1+(u,a)和T2+(b,v)两部分，处理后用(a,b)连接两棵子树。
  - 子树size超过阈值：找到树中最长的边(u,v)，切断这条边分成两棵子树。并且需要在合并时考虑进行refine（与FLUTE中相同）。

#### B. OA-FLUTE

* 查找表直接构建的steiner树包含两种违规情况：
  * 边被障碍完全阻挡：与A中方法类似，拆分成两个子树，处理后由障碍的边界合并。
  * steiner点被障碍覆盖：找到斯坦纳点与pin的连线和障碍边界的交点，相邻交点间的连线最长的一条不连，相邻的path经过的障碍顶点都加入pin所在的子树（每个障碍顶点最多加入两个不同的子树）。
  * 最终合并子树时OA-FLUTE会排除所有角顶点。

#### C. Fast Implementation with Obstacle Tree Data Structure

* 建立OBTree来更快的(logn)判断steiner树两种违规情况。
* 将障碍递归的平均分为左右或者上下两部分，对于每个斯坦纳点和边logn次就可以找到阻挡它的所有障碍。

## VI. OARSMT Generation

* 用L型连接两点时会有下面4种情况：
  1. 两种L路径都没有障碍：建立倾斜的连接边
  2. 两种L路径被同一障碍阻挡：绕路连接，选择较短的一条
  3. 两种L路径中有一条被障碍阻挡：被阻挡的一条分成两条递归的处理，若长度等于曼哈顿距离则选择被阻挡的，否则选择没有阻挡的
  4. 两种L路径被不同的障碍阻挡：选择更短的一条

## VII. Refinement

* 对于V型进行优化，对于3个点确定一个Steiner点，坐标是x坐标的中值和y坐标的中值





