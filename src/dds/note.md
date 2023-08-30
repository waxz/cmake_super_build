# cyclonedds
https://cyclonedds.io/docs/cyclonedds/latest/getting_started/helloworld/helloworld_building.html

# iceoryx
https://iceoryx.io/latest/


# use shared memory
https://cyclonedds.io/docs/cyclonedds/latest/shared_memory/shared_memory.html

### create `iox_config.toml`
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