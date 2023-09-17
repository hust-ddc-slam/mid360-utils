# mid360-utils

本代码为mid360使用的一些测试代码。

## mid360_viewer
**编译：**
正常clone下来，编译即可。由于需要livox的自定义消息类型，所以与`livox_ros_driver2`放到同一个 working_space 下的src中较为方便。
或者自行修改`CustomMsg.h`的路径也行。

**运行：**
```bash
roslaunch mid360_viewer viewer.launch
```

**说明：**
代码能够接收两种数据类型，分别是ros下的sensor_msg的pointcloud2，以及livox的自定义消息类型。
在launch文件中修改`msg_type`的类型，如果是pointcloud2格式，则代码直接收发，过程中计算了角度并存到intensity字段；
如果是`custom_msg`的类型，则分开4条线束的id并分开publish出来，同时将自定义消息类型中的`offset_time`字段放到intensity，并计算了角度。

有趣的发现：
- livox一次扫描时10hz，约20,000个点，其中大概5,000个为一个line；
- 每个line在非重复模式下扫描下，一个line并不是严格意义的一条ring，在空间中分布较为奇特，且`offset_time`在每条line中并不是同一圈递增的。

**TODO:** 测试重复扫描模式下的数据特征是什么样的。
