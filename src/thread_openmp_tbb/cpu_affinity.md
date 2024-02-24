# cpu arch
```shell
lscpu
```
```txt
Architecture:                    x86_64
CPU op-mode(s):                  32-bit, 64-bit
Byte Order:                      Little Endian
Address sizes:                   48 bits physical, 48 bits virtual
CPU(s):                          16
On-line CPU(s) list:             0-15
Thread(s) per core:              2
Core(s) per socket:              8
Socket(s):                       1
NUMA node(s):                    1
Vendor ID:                       AuthenticAMD
CPU family:                      23
Model:                           96
Model name:                      AMD Ryzen 7 4800H with Radeon Graphics
Stepping:                        1
Frequency boost:                 enabled
CPU MHz:                         1423.189
CPU max MHz:                     2900.0000
CPU min MHz:                     1400.0000
BogoMIPS:                        5789.29
Virtualization:                  AMD-V
L1d cache:                       256 KiB
L1i cache:                       256 KiB
L2 cache:                        4 MiB
L3 cache:                        8 MiB
NUMA node0 CPU(s):               0-1
```


# Cgroup v2
[在Ubuntu 20.04 LTS激活Cgroup v2 — Cloud Atlas beta 文档](https://cloud-atlas.readthedocs.io/zh-cn/latest/linux/ubuntu_linux/cgroup/enable_cgroup_v2_ubuntu_20.04.html)

# performance problem
- cache miss
- false sharing
- data racing

https://stackoverflow.com/questions/39495136/why-settting-cpu-affinity-make-threads-run-slower
https://stackoverflow.com/questions/18559342/what-is-a-cache-hit-and-a-cache-miss-why-would-context-switching-cause-cache-mi
