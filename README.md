使用C++重写的[Olympic引擎](https://github.com/jidiai/Competition_Olympics-Integrated)。

# 使用

有两种方法可以使用该代码。

第一种是使用engine部分的c++代码，scenario采用原来的包装。不同的scenario需要改动一部分代码，比如比较src/python/curling.py和src/olympics_engine/scenario/curling.py。

主要要改动的地方有：

-任何涉及到agent的更改都需要调用从c++中包装的函数，因为pybind11的转换机制不能很方便的改变std::vector。

-input action部分需要将None改为[0,0]

第二种是将scenario使用c++重写。比如代码中的curling类，这样可以方便地用envpool这些高性能并行环境库。


# 已知问题

get_obs函数中无法控制draw_line中元素的顺序和python版的一模一样（涉及到set的顺序），然后会导致DDA_line的结果相差一两个像素（这是原来代码的问题，没有写一个对顺序不敏感的渲染函数）

# Benchmark

在curling环境下第一种方法每秒约1700帧。作为对比python大约每秒230帧，快了接近8倍。
