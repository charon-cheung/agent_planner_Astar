1. 将所有package放到src文件夹下编译
2. `roslaunch planner plan.launch`是执行`planner`节点和`rivz`，`roslaunch agent agent.launch`是执行agent节点，二者顺序随意．
3. `rviz`中蓝线对应agent_1的路径，黄线对应agent_2的路径
4. 更改目标点，只能到`agent.launch`中修改param

题目要求的设计方式不合理，存在`get_plan`和`update_goal`两个service导致程序设计太繁杂，需要线程太多;两个agent存在同名service和topic也不合理，应该加节点名称做前缀;没必要一直发布起始点，因为没有变化;无法再扩展agent．

