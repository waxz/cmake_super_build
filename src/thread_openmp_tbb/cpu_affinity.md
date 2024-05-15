# set affinity in shell 
- https://stackoverflow.com/questions/13407813/shell-scripting-using-grep-to-split-a-string
- https://www.baeldung.com/linux/grep-exclude-ps-results
- https://www.baeldung.com/linux/trim-whitespace-bash-variable
- https://stackoverflow.com/questions/2159860/viewing-full-output-of-ps-command
- https://stackoverflow.com/questions/34669239/how-to-pass-command-with-parameters-to-xargs
- https://www.baeldung.com/linux/process-set-processor-affinity
- https://www.baeldung.com/linux/disable-cpu-cores
- https://www.baeldung.com/linux/managing-processors-availability
- 

### use taskset
https://man7.org/linux/man-pages/man1/taskset.1.html

OPTIONS         top

       -a, --all-tasks
           Set or retrieve the CPU affinity of all the tasks (threads)
           for a given PID.

       -c, --cpu-list
           Interpret mask as numerical list of processors instead of a
           bitmask. Numbers are separated by commas and may include
           ranges. For example: 0,5,8-11.

       -p, --pid
           Operate on an existing PID and do not launch a new task.

       -h, --help
           Display help text and exit.

       -V, --version
           Print version and exit.

USAGE         top

       The default behavior is to run a new command with a given
       affinity mask:
           taskset mask command [arguments]

       You can also retrieve the CPU affinity of an existing task:
           taskset -p pid

       Or set it:
           taskset -p mask pid

       When a cpu-list is specified for an existing process, the -p and
       -c options must be grouped together:
           taskset -pc cpu-list pid

       The --cpu-list form is applicable only for launching new
       commands:
           taskset --cpu-list cpu-list command


```shell
taskset -cpa 0,1 <tid>
```

```shell
ps -A -o tid,cmd  | grep -v grep | grep python | tr -s '[:blank:]' 

```
find core id
 

bind core
```shell
ps -A -o tid,cmd  | grep -v grep | grep _test | awk '{print $1}' | xargs -I {} /bin/bash -c ' echo taskset {}; taskset -cpa 0-2 {} ' 
```

### use Cgroup v2
- [在Ubuntu 20.04 LTS激活Cgroup v2 — Cloud Atlas beta 文档](https://cloud-atlas.readthedocs.io/zh-cn/latest/linux/ubuntu_linux/cgroup/enable_cgroup_v2_ubuntu_20.04.html)
- https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/8/html/managing_monitoring_and_updating_the_kernel/using-cgroups-v2-to-control-distribution-of-cpu-time-for-applications_managing-monitoring-and-updating-the-kernel#doc-wrapper





# performance problem
- cache miss
- false sharing
- data racing

https://stackoverflow.com/questions/39495136/why-settting-cpu-affinity-make-threads-run-slower
https://stackoverflow.com/questions/18559342/what-is-a-cache-hit-and-a-cache-miss-why-would-context-switching-cause-cache-mi
