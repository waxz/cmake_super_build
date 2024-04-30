# dependencies
https://fast-dds.docs.eprosima.com/en/latest/notes/versions.html#dependencies-compatibilities-library-dependencies

### ssl
https://gist.github.com/joulgs/c8a85bb462f48ffc2044dd878ecaa786
```shell
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.0g-2ubuntu4_amd64.deb
sudo dpkg -i libssl1.1_1.1.0g-2ubuntu4_amd64.deb
```
### fastddsgen build
https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#compiling-fast-dds-gen

```shell

cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
cd Fast-DDS-Gen
git checkout v3.3.0
./gradlew assemble
```
To make these scripts accessible from any shell session and directory, add the `scripts` folder path to the `PATH` environment variable.

# fastddsgen usage
```shell
fastddsgen -replace -example CMake ./HelloWorld.idl
```

# run

### SIMPLE
```shell
MY_APP_NAME=test_app ./dds_simple_participant -x ./dds_config.xml -p participant_discovery_protocol_SIMPLE 
```
### SIMPLE + ROS_DISCOVERY_SERVER
#### server
```shell
MY_APP_NAME=test_app_server ./dds_simple_participant -x ./dds_config.xml -p participant_discovery_protocol_SERVER
```
#### client
```shell
 ROS_DISCOVERY_SERVER="100.115.133.144:11811" MY_APP_NAME=test_app_1 ./dds_simple_participant -x ./dds_config.xml -p participant_discovery_protocol_SIMPLE
```



# benchmark
## best effort, no sleep
### on shm
**** Participant_sub_shm  band_width = 111176MB/s, rqs = 106026/s


### on udp lo, limit 12.8 Gbits/sec, 1638.4MB/s
**** Participant_sub_udp  band_width = 654.375MB/s, rqs = 624.061/s


### on udp eno1, limit 1.0 Gbits/sec, 128MB/s
**** Participant_sub_udp  band_width = 119.544MB/s, rqs = 114.006/s

## cpu, memory
### shm, 100 hz, 1MB package
1.5% cpu, 
### shm, 1000 hz, 1MB package
5.5% cpu,
### udp lo, 100 hz, 1MB package
65% cpu,
### udp lo, 100 hz, 1MB package
26% cpu,


### shm, 100 hz, 8KB package
1.5% cpu,
### shm, 1000 hz, 8KB package
6.5% cpu,

### udp lo, 100 hz, 8KB package
4.0% cpu,

### ros, 100 hz, 8KB package
2.5% cpu,


# problem in WI-FI 
- latency
- switch ap
filter WI-FI interface



# doc

### listening
https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html?highlight=metatrafficMulticastLocatorList

### discovery
https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/wifi/discovery_server_use_case.html#discovery-server

### qos
https://fast-dds.docs.eprosima.com/en/latest/fastdds/property_policies/ignore_local_endpoints.html
https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/qos.html
https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/troubleshooting.html

###  use case
https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/wifi/wifi.html
####  avoid multicast discovery
- SIMPLE: configure an initial list of remote peers on the DomainParticipant, so that it can set unicast communication with them.
- SERVER: Discovery Server can be used to avoid multicast discovery
- STATIC: If all the peers are known and configured beforehand, STATIC discovery can be used instead, completely avoiding the discovery phase




# reader
https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/dataReader/readingData.html#datareader-non-blocking-calls

# port
https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/domainparticipant.html#port-configuration

# binding port
https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html

# interface filter
https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/interfaces.html


# xml
https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/making_xml_profiles.html#loading-and-applying-profiles

In case the user defines the Entity profiles via XML files, it is required to load these XML files using the load_XML_profiles_file() public member function before creating any entity. It is also possible to load directly the XML information as a string data buffer using the load_XML_profiles_string() public member function. Moreover, create_participant_with_profile(), create_publisher_with_profile(), create_subscriber_with_profile(), create_datawriter_with_profile(), and create_datareader_with_profile() member functions expect a profile name as an argument. Fast DDS searches the given profile name over all the loaded XML profiles, applying the profile to the entity if founded.


# threads
https://fast-dds.docs.eprosima.com/en/latest/fastdds/library_overview/library_overview.html#concurrency-and-multithreading


# use toml config file

# use tiny alloc

# make code clean and simple

# reflection

```toml
[topic.hello_pub]
topic_name = "pub_msg"
topic_type = "String"

```


