编译流程
本方案基于BOOM 环境开发，使用前请对齐环境版本 chipyard1.7.0, 对应BOOM https://github.com/riscv-boom/riscv-boom/blob/ad64c5419151e5e886daee7084d8399713b46b4b。

vivado-riscv-v的commit id: commit c488d6221f06ba27cd2892b30b2027027b5f54c7。

在准备好之后先执行：

export PATH=$PATH:/opt/Xilinx/Vivado/2021.1/bin
将vivado加入环境变量，方便后续编译比特流。

之后进入vivado-riscv-v目录，并将你需要的BOOM代码替换掉路径vivado-risc-v/generators/riscv-boom/src/main/scala下的代码，最后执行：

make CONFIG=Rocket64x1 BOARD=genesys2 bitstream
等待编译结果，编译成功后会在路径vivado-risc-v/workspace/Rocket64x1/vivado-genesys2-riscv/genesys2-riscv.runs/impl_1/riscv_wrapper.bit下生成编译好的比特流文件，可以通过scp传输到主机，烧录到FPGA上进行测试。

需要注意的是，编译过程可能会比较长，具体时间取决于你的计算机性能和编译环境的配置，请耐心等待。

开发
本项目自带有BOOM-stop相关的代码，能直接通过修改计数器对各种事件进行记录。如果需要添加或修改新的事件计数，可以在src/main/scala-addi/exu/core.scala中找到相关的代码段，按照现有的模式添加新的事件计数逻辑。

相关项目链接：https://github.com/Shuiliusheng/boom_stop/tree/main 。

上板运行
可以将需要重复使用的文件放在SD卡上，避免反复传输。对于本实验来说，最重要的是将Linux系统及SPEC 2006及SPEC 2017相关的文件放在SD卡上。

在主机上拿到对应比特流后，使用vivado烧录工具将比特流烧录到FPGA上，之后通过串口连接FPGA，启动LINUX系统并运行相应的测试程序，即可在串口输出中看到事件计数器的结果。

运行SPEC目前有两类方法，有Simpoint和运行前一百亿条指令两类。Simpoint的反映的运行结果更有效，但需要制作切片。部分 SPEC 2006 和 2017 的切片见23服务器/home/chenkefa/disk/bishe/simpoint06 和/home/chenkefa/disk/bishe/extract_rate

根据bash脚本，选定模块直接运行即可获得log信息。

可以通过UART协议或者IP网络（FPGA如果连了网）传输到PC或服务器上，进行数据分析。
