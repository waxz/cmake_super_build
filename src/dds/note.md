# cyclonedds
https://cyclonedds.io/docs/cyclonedds/latest/getting_started/helloworld/helloworld_building.html

# iceoryx
https://iceoryx.io/latest/

# IDL
https://www.ibm.com/docs/en/integration-bus/9.0.0?topic=corba-idl-data-types

# use shared memory
https://cyclonedds.io/docs/cyclonedds/latest/shared_memory/shared_memory.html

server and client should use the same release, or download release v2.0.3
```shell
git checkout v2.0.3
```

# use iceoryx in docker
https://iceoryx.io/v2.0.2/examples/icedocker/


```shell

services:
  roudi:
    image: archlinux:latest
    command: /iceoryx/build/iox-roudi
    volumes:
      - .:/iceoryx
      - /dev:/dev
      - /tmp:/tmp
```

### create `iox_config.toml`

```
The following mempools are available:  MemPool [ ChunkSize = 1048640, ChunkPayloadSize =
1048600, ChunkCount = 10 ]Could not find a fitting mempool for a chunk of size 1048680
```
Chunk header is 40 byte;


`size` should be larger than message size plus header
for example: 16384 bytes (+ 64 byte header = 16448 byte block):

```toml
[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 1400
count = 1024
```
###  create `cyclonedds.xml`
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```
The configuration does not depend exclusively on the xml file. The content of the xml can be set directly into the envrionment variable CYCLONEDDS_URI.
```shell
export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery> <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints> </Discovery><Tracing><Verbosity>config</Verbosity><OutputFile>stdout</OutputFile></Tracing><SharedMemory><Enable>true</Enable><LogLevel>info</LogLevel></SharedMemory></Domain></CycloneDDS>"
```

### run 
in terminal 1
```shell
./force_iceoryx/install/bin/iox-roudi -c ../src/dds/iox_config.toml
```
in terminal 2
```shell
export CYCLONEDDS_URI=file:///home/waxz/CLionProjects/cmake_super_build/src/dds/cyclonedds.xml
./bin/ddscxxPing
```


# qos
https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html

https://cyclonedds.io/content/faq.html
https://github.com/eclipse-cyclonedds/cyclonedds/issues/78

# dds_take vs dds_read
https://github.com/eclipse-cyclonedds/cyclonedds/issues/17


# take with shared memory or socket
https://github.com/eclipse-cyclonedds/cyclonedds/issues/785


# performance
c++ problem
https://github.com/eclipse-cyclonedds/cyclonedds-cxx/issues/358

sharedmemory 
https://github.com/eclipse-cyclonedds/cyclonedds/issues/785










# FastDDS
https://discourse.ros.org/t/eprosima-fast-dds-from-shared-memory-to-zero-copy/18877
https://discourse.ros.org/t/fast-dds-v2-2-0-throughput-performance/19218
https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/zero_copy/zero_copy.html
https://github.com/eProsima/Fast-DDS/issues/2343
# benchmark

https://discourse.ros.org/t/new-fast-dds-performance-testing/29539/6


## cyclone dds 
### c write, c++ read, shm
- band_width = 60025.2MB/s
- rps = 57245.1request/s

### c write, c read, shm
- band_width = 10009.712640 MB/s
- rps = 9546.005859/s
- writer is faster than reader, may crash because of  run out of memory pool chunks

### c++ write, c++ read, shm
- band_width = 126909MB/s
- rps = 121031request/s



## fastdds 

### tcp
- band_width = 231.841MB/s
- rps = 221.103request/s

### zero copy sub qos RELIABLE_RELIABILITY_QOS
- band_width = 46925.7MB/s
- rps = 44747.7request/s
### zero copy sub qos BEST_EFFORT_RELIABILITY_QOS
- band_width = 118393MB/s
- ps = 112909request/s



# toml
doc https://toml.io/en/
cheatsheet https://quickref.me/toml.html
cheatsheet https://learnxinyminutes.com/docs/toml/
Online TOML to JSON converter https://toml-to-json.matiaskorhonen.fi/


# IDL

```shell
fastddsgen -replace ./ShmThroughput.idl 
```
`-replace` 覆盖已存在的文件


# fastdds_help_test
```shell
TCC_HOME=/tmp/cmake_super_build/cmake-build-relwithdebinfo/force_tinycc/install;TCC_OPTION=-std=c11;TCC_HEADER=/tmp/cmake_super_build/src/c_test/
```