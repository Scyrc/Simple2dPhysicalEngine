# Simple2dPhysicalEngine

## 一个简单的2d物理引擎

### 内容
- 基本2D几何计算,2D图元(凸多面体，圆)
- 2D图元碰撞检测-broadphase: AABB包围盒， 动态层次包围体树DBVH
- 2D图元碰撞检测-narrowphase: SAT算法 ，多边形裁剪进行接触点计算
- 碰撞解算：基于冲量的刚体动力学
- UI界面:使用opengl绘制图形，imgui界面管理，用户交互

### 场景
- #### fallDown
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/fallDown.gif)

- #### restitution
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/restitution.gif)

- #### friction
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/friction.gif)

- #### pyramid
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/pyramid.gif)

- #### newton's_pendulum
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/newton's_pendulum.gif)

- #### bridge
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/bridge.gif)


- #### wreckingBall
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/wreckingBall.gif)


- #### domino
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/domino.gif)

- #### seesaw
![image](https://github.com/Scyrc/Simple2dPhysicalEngine/blob/master/gif/seesaw.gif)