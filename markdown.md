<!-- This is the right way to write markdown comment -->
<!-- /TOC -->
<!-- [碰撞检测原理及算法实现](#碰撞检测原理及算法实现)
- [BV包围体](#BV包围体)
- [SAT分离轴理论](#SAT分离轴理论)
- [GJK算法](#gjk算法)
- [参考文献](#参考文献)
- [备注(如何描述三维物体)](#备注如何描述三维物体) -->


# 一、碰撞检测原理及算法实现

## BV包围体
- 包围球Spheres

- 轴对齐包围盒AABB

- 有向包围盒OBB


## SAT分离轴理论
- 原理
>**两个多边形在所有轴上的投影都发生重叠，则判定为碰撞；否则，没有发生碰撞。**

- 投影

<center>
<img src="http://oj6n9xi25.bkt.clouddn.com/17-1-29/54550340-file_1485662433642_1cc2.png" width="40%" />
图.两个三角形碰撞检测
</center>

- 代码实现
```java {.line-numbers}
close all;clear;clc
close all;clear;clc
% 伪代码实现
double overlap = // really large value;
Axis smallest = null;
Axis[] axes1 = shape1.getAxes();
Axis[] axes2 = shape2.getAxes();
// loop over the axes1
for (int i = 0; i < axes1.length; i++) {
  Axis axis = axes1[i];
  // project both shapes onto the axis
  Projection p1 = shape1.project(axis);
  Projection p2 = shape2.project(axis);
  // do the projections overlap?
  if (!p1.overlap(p2)) {
    // then we can guarantee that the shapes do not overlap
    return false;
  } else {
    // get the overlap
    double o = p1.getOverlap(p2);
    // check for minimum
    if (o < overlap) {
      // then set this one as the smallest
      overlap = o;
      smallest = axis;
    }
  }
}
// loop over the axes2
for (int i = 0; i < axes2.length; i++) {
  Axis axis = axes2[i];
  // project both shapes onto the axis
  Projection p1 = shape1.project(axis);
  Projection p2 = shape2.project(axis);
  // do the projections overlap?
  if (!p1.overlap(p2)) {
    // then we can guarantee that the shapes do not overlap
    return false;
  } else {
    // get the overlap
    double o = p1.getOverlap(p2);
    // check for minimum
    if (o < overlap) {
      // then set this one as the smallest
      overlap = o;
      smallest = axis;
    }
  }
}
MTV mtv = new MTV(smallest, overlap);
// if we get here then we know that every axis had overlap on it
// so we can guarantee an intersection
return mtv;
```

## GJK算法

- 原理
>只对凸体有效；支持任何凸体形状之间的碰撞检测；GJK是一个迭代算法，如果事先给出分离向量，它的收敛会很快，可以在常量时间内完成；在3D环境中，GJK可以取代SAT算法。

凸体：其实就是一条直线穿越凸体，和该凸体壳的交点不能超过2个。

闵科夫斯基和：假设有两个物体，它们的闵科夫斯基和就是物体1上的所有点和物体2上的所有点的和集。用公式表示就是： A + B = {a + b | a∈A, b∈B}

>如果两个物体都是凸体，它们的明可夫斯基和也是凸体。

闵科夫斯基差：A – B = A + (-B) = {a + (– b)|a∈A, b∈B} = {a – b)|a∈A, b∈B}

>**如果两个物体重叠或者相交，它们的闵科夫斯基差肯定包括原点。**

闵科夫斯基差操作需要物体1的顶点数*物体2的顶点数*2(对于二维向量为2，如果在三维空间，当然就是*3了，如果是向量减法数量就什么都不用乘了) 个减法操作。物体包含无穷多个点，但由于是凸体，我们可以只对它们的顶点执行闵科夫斯基差操作。

<center>
<img src="http://oj6n9xi25.bkt.clouddn.com/17-1-29/57010667-file_1485666583149_14aae.png " width="40%" />
  
<img src="http://oj6n9xi25.bkt.clouddn.com/17-1-29/4670185-file_1485666689361_14785.png " width="40%" />
图.两个三角形碰撞检测
</center>

- 单纯形


- 相交检测


- 代码实现
```javascript {.line-numbers}
 close all;clear;clc
% 伪代码实现
 Vector d = // choose a search direction
 // get the first Minkowski Difference point
 Simplex.add(support(A, B, d));
 // negate d for the next point
 d.negate();
 // start looping
 while (true) {
   // add a new point to the simplex because we haven't terminated yet
   Simplex.add(support(A, B, d));
   // make sure that the last point we added actually passed the origin
   if (Simplex.getLast().dot(d) <= 0) {
     // if the point added last was not past the origin in the direction of d
     // then the Minkowski Sum cannot possibly contain the origin since
     // the last point added is on the edge of the Minkowski Difference
     return false;
   } else {
     // otherwise we need to determine if the origin is in
     // the current simplex
     if (containsOrigin(Simplex, d) {
       // if it does then we know there is a collision
       return true;
     }
   }
 }

 public boolean containsOrigin(Simplex s, Vector d) {
   // get the last point added to the simplex
   a = Simplex.getLast();
   // compute AO (same thing as -A)
   ao = a.negate();
   if (Simplex.points.size() == 3) {
     // then its the triangle case
     // get b and c
     b = Simplex.getB();
     c = Simplex.getC();
     // compute the edges
     ab = b - a;
     ac = c - a;
     // compute the normals
     abPerp = tripleProduct(ac, ab, ab);
     acPerp = tripleProduct(ab, ac, ac);
     // is the origin in R4
     if (abPerp.dot(ao) > 0) {
       // remove point c
       Simplex.remove(c);
       // set the new direction to abPerp
       d.set(abPerp);
     } else {
       // is the origin in R3
       if (acPerp.dot(ao) > 0) {
         // remove point b
         Simplex.remove(b);
         // set the new direction to acPerp
         d.set(acPerp);
       } else{
         // otherwise we know its in R5 so we can return true
         return true;
       }
     }
   } else {
     // then its the line segment case
     b = Simplex.getB();
     // compute AB
     ab = b - a;
     // get the perp to AB in the direction of the origin
     abPerp = tripleProduct(ab, ao, ab);
     // set the direction to abPerp
     d.set(abPerp);
   }
   return false;
 }
```

## 参考文献
[1][《实时碰撞检测算法技术》读书笔记(一)：包围体（BV）](http://blog.csdn.net/u010387196/article/details/19207131)
[2][ 碰撞检测之分离轴定理算法讲解](http://blog.csdn.net/yorhomwang/article/details/54869018)
[3][ 判断是两个形状是否相交(一)-SAT分离轴理论](http://blog.csdn.net/u011373710/article/details/54773171)
[4][ 判断两个形状是否相交(二)-GJK算法](http://blog.csdn.net/u011373710/article/details/54773174)

***
## 备注（如何描述三维物体）
 - 包围体
 - 像素化（voxel.m）
 - 三角网格(只描述表面；代码C:\Users\stevewen\文档\MATLAB\distmesh)
```Matlab {.line-numbers}
fd=@(p)(sum(p.^2,2)+.8^2-.2^2).^2-4*.8^2*(p(:,1).^2+p(:,2).^2);
[p,t]=distmeshsurface(fd,@huniform,0.1,[-1.1,-1.1,-.25;1.1,1.1,.25]);
```


# 二、路径规划
## 算法原理


## 算法实现


## 参考文献

## 备注
