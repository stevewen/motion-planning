<!-- TOC depthFrom:1 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [碰撞检测原理及算法实现](#碰撞检测原理及算法实现)
	- [包围体BV](#包围体bv)
	- [分离轴理论SAT](#分离轴理论sat)
	- [GJK算法](#gjk算法)
	- [参考文献](#参考文献)
	- [备注（如何描述三维物体）](#备注如何描述三维物体)

<!-- /TOC -->

# 碰撞检测原理及算法实现

## 包围体BV
- 包围球Spheres

- 轴对齐包围盒AABB

- 有向包围盒OBB


## 分离轴理论SAT
- 原理
>**两个多边形在所有轴上的投影都发生重叠，则判定为碰撞；否则，没有发生碰撞。**

- 投影

<center>
<img src="http://oj6n9xi25.bkt.clouddn.com/17-1-29/54550340-file_1485662433642_1cc2.png" width="30%" />
图  .两个三角形碰撞检测
</center>

- 代码实现

 ```Matlab
 close all;clear;clc

 ```

## GJK算法
- 原理

>**只对凸体有效；支持任何凸体形状之间的碰撞检测。**

- 代码实现

 ```Matlab
 close all;clear;clc

 ```

## 参考文献
1. [《实时碰撞检测算法技术》读书笔记（一）：包围体（BV）](http://blog.csdn.net/u010387196/article/details/19207131)
2. [碰撞检测之分离轴定理算法讲解](http://blog.csdn.net/yorhomwang/article/details/54869018?locationNum=5&fps=1)
3. [判断是两个形状是否相交(一)-SAT分离轴理论](http://blog.csdn.net/u011373710/article/details/54773171)
4. [判断两个形状是否相交(二)-GJK算法](http://blog.csdn.net/u011373710/article/details/54773174)

***
## 备注（如何描述三维物体）
 - 包围体
 - 像素化（voxel.m）
 - 三角网格(只描述表面；代码C:\Users\stevewen\文档\MATLAB\distmesh)
```Matlab
fd=@(p)(sum(p.^2,2)+.8^2-.2^2).^2-4*.8^2*(p(:,1).^2+p(:,2).^2);
[p,t]=distmeshsurface(fd,@huniform,0.1,[-1.1,-1.1,-.25;1.1,1.1,.25]);
```
